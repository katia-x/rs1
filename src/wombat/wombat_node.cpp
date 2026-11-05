#include "wombat_node.h"

#include <cmath>
#include <functional>
#include <stdexcept>

using namespace std::chrono_literals;
using GoalRequest = rs1::srv::WombatNextGoal::Request;

WombatNode::WombatNode()
: Node("wombat_node"),
	platform_(this->get_logger()),
  global_frame_(declare_parameter("global_frame", "map")),
  lane_start_wait_duration_(declare_parameter("lane_start_wait_duration", 1.0)),
  max_goal_retries_(declare_parameter("max_goal_retries", 3)),
  nav_unavailable_timeout_(declare_parameter("nav_unavailable_timeout", 60.0))
{
	if (!std::isfinite(lane_start_wait_duration_) || lane_start_wait_duration_ < 0.0) {
		throw std::invalid_argument("lane_start_wait_duration must be finite and non-negative");
	}
	if (max_goal_retries_ < 0) {
		throw std::invalid_argument("max_goal_retries must be non-negative");
	}
	if (!std::isfinite(nav_unavailable_timeout_) || nav_unavailable_timeout_ <= 0.0) {
		throw std::invalid_argument("nav_unavailable_timeout must be finite and positive");
	}
	mission_client_ = create_client<rs1::srv::WombatNextGoal>("wombat_next_goal");
	delete_entity_client_ = create_client<DeleteEntity>(
		"/world/rs1_terrain/remove");
	weed_goal_subscription_ = create_subscription<rs1::msg::WombatWeedGoal>(
		"/wombat/weed_goal", rclcpp::QoS(1).transient_local(),
		std::bind(&WombatNode::weed_goal_callback, this, std::placeholders::_1));
	odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
		"/husky1/odometry", rclcpp::QoS(10),
		std::bind(&WombatNode::odometry_callback, this, std::placeholders::_1));
	status_subscription_ = create_subscription<rs1::msg::WombatStatus>(
		"/wombat/status", rclcpp::QoS(1).transient_local(),
		std::bind(&WombatNode::status_callback, this, std::placeholders::_1));
	state_publisher_ = create_publisher<rs1::msg::WombatState>(
		"/wombat/state", rclcpp::QoS(1).transient_local());
	trajectory_goals_publisher_ = create_publisher<rs1::msg::GliderGoal>(
		"/glider/trajectory_goals", rclcpp::QoS(10));
	navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
		this, declare_parameter("navigation_action", "/husky1/navigate_to_pose"));

	weeds_removed_publisher_ = create_publisher<std_msgs::msg::UInt32>(
		"/wombat/weeds_removed",
		rclcpp::QoS(1).reliable().transient_local());

	// Publish the initial zero.
	std_msgs::msg::UInt32 initial_count;
	initial_count.data = 0;
	weeds_removed_publisher_->publish(initial_count);
    
	gui_management_ = std::make_unique<GuiManagement>(
		*this,
		[this](bool enabled) { handle_override(enabled); },
		[this]() { handle_estop(); },
		[this]() { handle_resume(); });

	mission_retry_timer_ = create_wall_timer(100ms, [this]() { timer_callback(); });
	state_timer_ = create_wall_timer(1s, [this]() { publish_state(); });
	publish_state();
}

void WombatNode::publish_state()
{
	rs1::msg::WombatState message;
	message.state = static_cast<std::uint8_t>(state_);
	message.mission_generation = mission_generation_;
	state_publisher_->publish(message);
}

void WombatNode::set_state(State state)
{
	if (state_ == state) {
		return;
	}
	state_ = state;
	publish_state();
}

void WombatNode::try_delete_weed()
{
	if (weed_deletion_in_flight_) {
		return;
	}
	if (last_weed_name_.empty()) {
		finish_weed_removal(true);
		return;
	}
	if (weed_deletion_attempts_ >= 3) {
		finish_weed_removal(false);
		return;
	}
	if (!delete_entity_client_->service_is_ready()) {
		++weed_deletion_attempts_;
		RCLCPP_WARN(get_logger(), "Weed deletion service unavailable for %s (attempt %d/3)",
			last_weed_name_.c_str(), weed_deletion_attempts_);
		if (weed_deletion_attempts_ >= 3) {
			finish_weed_removal(false);
			return;
		}
		weed_deletion_retry_pending_ = true;
		return;
	}

	weed_deletion_retry_pending_ = false;
	weed_deletion_in_flight_ = true;
	++weed_deletion_attempts_;

	const auto model_name = last_weed_name_;
	const auto request_id = ++deletion_request_id_;

	auto request = std::make_shared<DeleteEntity::Request>();
	request->entity.name = model_name;
	request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;

	deletion_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);

	const auto pending = delete_entity_client_->async_send_request(
		request,
		[this, model_name, request_id](
			rclcpp::Client<DeleteEntity>::SharedFuture future)
		{
			bool removed = false;
			try {
				removed = future.get()->success;
			} catch (const std::exception & error) {
				RCLCPP_WARN(
					get_logger(), "Deletion request for %s failed: %s",
					model_name.c_str(), error.what());
			}

			// Count confirmed deletions, including late successful responses.
			if (removed && removed_plant_names_.insert(model_name).second) {
				RCLCPP_INFO(
					get_logger(), "Removed %s; total plants removed: %zu",
					model_name.c_str(), removed_plant_names_.size());

				std_msgs::msg::UInt32 count;
				count.data = static_cast<std::uint32_t>(removed_plant_names_.size());
				weeds_removed_publisher_->publish(count);
			}

			// An old response must not change the current mission.
			if (request_id != deletion_request_id_ || mission_stopped()) {
				return;
			}

			weed_deletion_in_flight_ = false;

			if (removed) {
				finish_weed_removal(true);
			} else if (weed_deletion_attempts_ < 3) {
				weed_deletion_retry_pending_ = true;
			} else {
				RCLCPP_ERROR(
					get_logger(), "Failed to delete %s after 3 attempts",
					model_name.c_str());
				finish_weed_removal(false);
			}
		});
		
	deletion_ros_request_id_ = pending.request_id;
}

void WombatNode::finish_weed_removal(bool removed)
{
	weed_deletion_in_flight_ = false;
	weed_deletion_retry_pending_ = false;
	if (removed) {
		queue_request(GoalRequest::WEED_HANDLED, last_weed_id_);
	} else {
		queue_request(GoalRequest::WEED_UNREACHABLE, last_weed_id_);
	}
	spawn_poo_if_due();
	last_goal_is_weed_ = false;
	last_weed_name_.clear();
	state_ = State::Searching;
}

void WombatNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message)
{
	latest_husky_odometry_ = *message;
}

void WombatNode::spawn_poo_if_due()
{
	++weeds_since_last_poo_;
	if (weeds_since_last_poo_ < 3) {
		return;
	}

	const auto & position = latest_husky_odometry_.pose.pose.position;
	const auto model_name = "poo_" + std::to_string(++poo_count_);
	platform_.spawnPoo(model_name, position.x, position.y, position.z);

	rs1::msg::GliderGoal goal;
	goal.x = position.x;
	goal.y = position.y;
	goal.z = 0.0;
	goal.model_name = model_name;
	trajectory_goals_publisher_->publish(goal);
	weeds_since_last_poo_ = 0;
	RCLCPP_INFO(get_logger(), "Spawned %s at Husky position (%.2f, %.2f, %.2f)",
		model_name.c_str(), position.x, position.y, position.z);
}

void WombatNode::timer_callback()
{
	if (mission_stopped()) {
		return;
	}

	if (weed_deletion_in_flight_) {
		if (std::chrono::steady_clock::now() < deletion_deadline_) {
			return;
		}

		RCLCPP_WARN(
			get_logger(), "Deletion timed out for %s (attempt %d/3)",
			last_weed_name_.c_str(), weed_deletion_attempts_);

		// Release the unanswered ROS request and invalidate its callback.
		delete_entity_client_->remove_pending_request(
			deletion_ros_request_id_);
		++deletion_request_id_;
		weed_deletion_in_flight_ = false;

		if (weed_deletion_attempts_ < 3) {
			weed_deletion_retry_pending_ = true;
		} else {
			finish_weed_removal(false);
		}
		return;
	}

	if (gui_management_->manual_override() || gui_management_->estop_active() || mission_request_in_flight_ ||
		nav_goal_request_in_flight_ || nav_cancel_in_flight_) {
		return;
	}
	if (weed_deletion_retry_pending_) {
		try_delete_weed();
		return;
	}

	// Keep the saved goal and deadline intact while waiting, even if new weeds arrive.
	if (nav_unavailable) {
		const auto elapsed = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - nav_unavailable_since_);

		if (elapsed.count() >= nav_unavailable_timeout_) {
			abort_mission();
			return;
		}

		if (last_navigation_goal_valid_ && navigation_client_->action_server_is_ready()) {
			send_navigation_goal(
				last_navigation_goal_,
				last_goal_is_weed_,
				last_weed_id_,
				last_weed_name_);
		}

		return;
	}

	if (pending_weed_goal_.pending && state_ == State::Searching) {
		if (canceling_goal_) { return; }
		if (active_goal_) {
			RCLCPP_INFO(get_logger(), "Canceling lane goal for weed %llu",
				static_cast<unsigned long long>(pending_weed_goal_.goal.weed_id));
			canceling_goal_ = active_goal_;
			nav_cancel_in_flight_ = true;
			navigation_client_->async_cancel_goal(
				canceling_goal_, std::bind(&WombatNode::handle_nav_cancel, this, std::placeholders::_1));
			return;
		}
		dispatch_pending_weed_goal();
		return;
	}
	if (active_goal_) { return; }

	switch (state_) {
	case State::Complete:
	case State::Abort:
		return;
	case State::PreLaneWait:
		if (std::chrono::steady_clock::now() < lane_wait_until_) { return; }
		set_state(State::Searching);
		[[fallthrough]];
	case State::WaitingForRegion:
		if (!pending_request_.pending) { pending_request_.pending = true; }
		break;
	case State::Searching:
		if (!pending_request_.pending) { queue_request(GoalRequest::REQUEST_NEXT_GOAL); }
		break;
	case State::Recovery:
		if (!pending_request_.pending) { queue_request(GoalRequest::SKIP_CURRENT_GOAL); }
		break;
	case State::ApproachingStart:
	case State::Turning:
	default:
		break;
	}

	if (!pending_request_.pending) {
		return;
	}

	request_next_goal(pending_request_.operation, pending_request_.weed_id);
}

void WombatNode::queue_request(std::uint8_t operation, std::uint64_t weed_id)
{
	pending_request_.pending = true;
	pending_request_.operation = operation;
	pending_request_.weed_id = weed_id;
}

void WombatNode::request_next_goal(std::uint8_t operation, std::uint64_t weed_id)
{
	if (mission_stopped()) {
		return;
	}
	if (active_goal_ || mission_request_in_flight_ || nav_goal_request_in_flight_) { return; }
	if (!mission_client_->wait_for_service(1s)) {
		RCLCPP_WARN(get_logger(), "Mission service is not available yet");
		return;
	}
	pending_request_.pending = false;
	mission_request_in_flight_ = true;
	mission_request_operation_ = operation;
	mission_request_generation_ = mission_generation_;
	const auto request_id = ++mission_request_id_;
	const auto request_override_generation = gui_management_->override_generation();
	auto request = std::make_shared<rs1::srv::WombatNextGoal::Request>();
	request->operation = operation;
	request->weed_id = weed_id;
	mission_client_->async_send_request(
		request, [this, request_id, request_override_generation](
			rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future) {
			handle_mission_goal(future, request_id, request_override_generation);
		});
}

void WombatNode::weed_goal_callback(const rs1::msg::WombatWeedGoal::SharedPtr message)
{
	if (mission_stopped() || message->mission_generation < mission_generation_) {
		return;
	}
	if (message->mission_generation > mission_generation_) {
		mission_generation_ = message->mission_generation;
		reset_for_new_region();
	}
	if (last_goal_is_weed_) {
		if (last_weed_id_ == message->weed_id) {
			return;
		}
		RCLCPP_DEBUG(get_logger(),
			"Deferring weed goal %llu while pursuing weed %llu",
			static_cast<unsigned long long>(message->weed_id),
			static_cast<unsigned long long>(last_weed_id_));
		return;
	}
	if (pending_weed_goal_.pending && pending_weed_goal_.goal.weed_id != message->weed_id) {
		RCLCPP_DEBUG(get_logger(),
			"Deferring weed goal %llu while weed %llu is pending",
			static_cast<unsigned long long>(message->weed_id),
			static_cast<unsigned long long>(pending_weed_goal_.goal.weed_id));
		return;
	}
	RCLCPP_INFO(get_logger(), "Received weed goal %llu for mission generation %llu",
		static_cast<unsigned long long>(message->weed_id),
		static_cast<unsigned long long>(message->mission_generation));
	pending_weed_goal_.goal = *message;
	pending_weed_goal_.pending = true;
}

void WombatNode::dispatch_pending_weed_goal()
{
	if (mission_stopped() || !pending_weed_goal_.pending || active_goal_ || nav_cancel_in_flight_) {
		return;
	}
	const auto weed_goal = pending_weed_goal_.goal;
	pending_weed_goal_.pending = false;
	set_state(State::Searching);
	RCLCPP_INFO(get_logger(), "Sending weed goal %llu to Nav2",
		static_cast<unsigned long long>(weed_goal.weed_id));
	goal_failures_ = 0;
	send_navigation_goal(weed_goal.goal, true, weed_goal.weed_id, weed_goal.weed_name);
}

void WombatNode::handle_nav_cancel(
	rclcpp_action::Client<NavigateToPose>::CancelResponse::SharedPtr response)
{
	nav_cancel_in_flight_ = false;
	if (!canceling_goal_) { return; }
	if (!response || response->return_code != action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
		RCLCPP_WARN(get_logger(), "Nav2 rejected cancellation of the lane goal");
		canceling_goal_.reset();
		return;
	}
	RCLCPP_INFO(get_logger(), "Nav2 accepted cancellation; waiting for the active goal to finish");
}

void WombatNode::complete_mission()
{
	if (active_goal_) {
		navigation_client_->async_cancel_goal(active_goal_);
		active_goal_.reset();
	}

	nav_unavailable = false;
	nav_unavailable_since_ = {};
	canceling_goal_.reset();
	nav_cancel_in_flight_ = false;
	set_state(State::Complete);
	pending_weed_goal_.pending = false;
	pending_request_.pending = false;
}

void WombatNode::status_callback(const rs1::msg::WombatStatus::SharedPtr message)
{
	if (message->mission_generation < mission_generation_) { return; }
	if (message->mission_generation > mission_generation_ || message->region_reset) {
		mission_generation_ = message->mission_generation;
		reset_for_new_region();
	}
	if (message->mission_complete && !mission_stopped()) {
		complete_mission();
	}
}

void WombatNode::reset_for_new_region()
{
	if (active_goal_) {
		navigation_client_->async_cancel_goal(active_goal_);
		active_goal_.reset();
	}

	if (weed_deletion_in_flight_) {
		delete_entity_client_->remove_pending_request(
			deletion_ros_request_id_);
	}

	++deletion_request_id_;
	nav_unavailable = false;
	nav_unavailable_since_ = {};
	canceling_goal_.reset();
	nav_cancel_in_flight_ = false;
	pending_weed_goal_.pending = false;
	pending_request_ = PendingRequest{};
	mission_request_operation_ = rs1::srv::WombatNextGoal::Request::REQUEST_NEXT_GOAL;
	goal_failures_ = 0;
	last_goal_is_weed_ = false;
	last_weed_id_ = 0;
	last_weed_name_.clear();
	weed_deletion_in_flight_ = false;
	weed_deletion_retry_pending_ = false;
	weed_deletion_attempts_ = 0;
	last_navigation_goal_valid_ = false;
	set_state(State::WaitingForRegion);
}

void WombatNode::send_navigation_goal(
	const geometry_msgs::msg::PoseStamped & pose, bool is_weed, std::uint64_t weed_id,
	const std::string & weed_name)
{
	if (mission_stopped()) {
		return;
	}

	NavigateToPose::Goal goal;
	goal.pose = pose;
	goal.pose.header.frame_id = goal.pose.header.frame_id.empty() ? global_frame_ : goal.pose.header.frame_id;
	goal.pose.header.stamp = now();
	if (is_weed) {
		if (!last_goal_is_weed_ || weed_name != last_weed_name_) {
			weed_deletion_attempts_ = 0;
		}
		last_weed_id_ = weed_id;
		last_weed_name_ = weed_name;
	}
	last_goal_is_weed_ = is_weed;

	last_navigation_goal_ = goal.pose;
	last_navigation_goal_valid_ = true;

	if (!navigation_client_->action_server_is_ready()) {
		if (!nav_unavailable) {
			nav_unavailable_since_ = std::chrono::steady_clock::now();
			RCLCPP_WARN(
				get_logger(), "Waiting up to %.1f seconds for Nav2 action server",
				nav_unavailable_timeout_);
		}
		nav_unavailable = true;
		return;
	}

	nav_unavailable = false;
	nav_unavailable_since_ = {};

	rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
	const auto goal_generation = mission_generation_;
	const auto request_id = ++nav_goal_request_id_;
	const auto request_override_generation = gui_management_->override_generation();
	options.goal_response_callback =
		[this, goal_generation, request_id, request_override_generation](NavigateGoalHandle::SharedPtr goal_handle) {
			if (request_id != nav_goal_request_id_) {
				if (goal_handle) { navigation_client_->async_cancel_goal(goal_handle); }
				return;
			}
			if (goal_generation != mission_generation_ ||
				request_override_generation != gui_management_->override_generation() ||
				mission_stopped()) {
				nav_goal_request_in_flight_ = false;
				if (goal_handle) { navigation_client_->async_cancel_goal(goal_handle); }
				return;
			}
			handle_nav_goal_response(goal_handle);
		};
	options.result_callback =
		[this, goal_generation, request_override_generation](const NavigateGoalHandle::WrappedResult & result) {
			if (goal_generation == mission_generation_ &&
				request_override_generation == gui_management_->override_generation() &&
				!mission_stopped()) {
				handle_nav_result(result);
			}
		};
	nav_goal_request_in_flight_ = true;
	navigation_client_->async_send_goal(goal, options);
}

void WombatNode::handle_mission_goal(
	rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future,
	std::uint64_t request_id, std::uint64_t request_override_generation)
{
	if (request_id != mission_request_id_) {
		return;
	}
	mission_request_in_flight_ = false;
	if (mission_stopped() || mission_request_generation_ != mission_generation_ ||
		request_override_generation != gui_management_->override_generation()) {
		return;
	}

	try {
		auto response = future.get();
		if (response->mission_complete) {
			complete_mission();
			return;
		}
		if (gui_management_->manual_override() || gui_management_->estop_active()) {
			return;
		}
		if (!response->success) {
			set_state(State::WaitingForRegion);
			pending_request_.pending = true;
			return;
		}
		if (mission_request_operation_ != rs1::srv::WombatNextGoal::Request::RETRY_CURRENT_GOAL) {
			goal_failures_ = 0;
		}
		RCLCPP_INFO(get_logger(),
			"Mission response operation=%u goal=(%.2f, %.2f) weed=%s turn=%s lane_start=%s",
			static_cast<unsigned>(mission_request_operation_),
			response->goal.pose.position.x, response->goal.pose.position.y,
			response->is_weed ? "true" : "false", response->is_turn ? "true" : "false",
			response->is_lane_start ? "true" : "false");
		if (response->is_lane_start) {
			set_state(State::ApproachingStart);
		} else if (response->is_turn) {
			set_state(State::Turning);
		} else {
			set_state(State::Searching);
		}
		send_navigation_goal(response->goal, response->is_weed, response->weed_id);
	} catch (const std::exception & error) {
		mission_request_in_flight_ = false;
		RCLCPP_WARN(get_logger(), "Mission service call failed: %s", error.what());
		set_state(State::Recovery);
		queue_request(mission_request_operation_, last_weed_id_);
	}
}

void WombatNode::handle_nav_goal_response(NavigateGoalHandle::SharedPtr goal_handle)
{
	nav_goal_request_in_flight_ = false;
	if (!goal_handle) {
		RCLCPP_ERROR(get_logger(), "Nav2 rejected the goal");
		set_state(State::Recovery);
		handle_nav_failure();
		return;
	}
	if (gui_management_->manual_override() || gui_management_->estop_active()) {
		navigation_client_->async_cancel_goal(goal_handle);
		return;
	}

	active_goal_ = goal_handle;
	RCLCPP_INFO(get_logger(), "Nav2 accepted the goal");
}

void WombatNode::handle_nav_result(const NavigateGoalHandle::WrappedResult & result)
{
	if (canceling_goal_ && result.goal_id == canceling_goal_->get_goal_id()) {
		nav_cancel_in_flight_ = false;
		active_goal_.reset();
		canceling_goal_.reset();
		RCLCPP_INFO(get_logger(), "Canceled navigation goal finished with result_code=%d",
			static_cast<int>(result.code));
		if (!mission_stopped()) { dispatch_pending_weed_goal(); }
		return;
	}
	if (nav_cancel_in_flight_) { return; }
	if (nav_goal_request_in_flight_ || !active_goal_ ||
		result.goal_id != active_goal_->get_goal_id()) { return; }
	active_goal_.reset();
	if (gui_management_->manual_override() || gui_management_->estop_active()) {
		return;
	}
	if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_INFO(get_logger(), "Nav2 reached the goal");
		goal_failures_ = 0;
		if (last_goal_is_weed_) {
			try_delete_weed();
			queue_request(GoalRequest::WEED_HANDLED, last_weed_id_);
			set_state(State::Searching);
		} else if (state_ == State::ApproachingStart || state_ == State::Turning) {
			set_state(State::PreLaneWait);
			lane_wait_until_ = std::chrono::steady_clock::now() +
				std::chrono::duration_cast<std::chrono::steady_clock::duration>(
					std::chrono::duration<double>(lane_start_wait_duration_));
			queue_request(GoalRequest::REQUEST_NEXT_GOAL);
		} else {
			queue_request(GoalRequest::REQUEST_NEXT_GOAL);
		}
		return;
	}
		RCLCPP_WARN(get_logger(), "Nav2 failed to reach the goal, result_code=%d",
		static_cast<int>(result.code));
	set_state(State::Recovery);
	handle_nav_failure();
}

void WombatNode::handle_nav_failure()
{
	++goal_failures_;
	if (goal_failures_ <= static_cast<std::size_t>(max_goal_retries_)) {
		RCLCPP_WARN(get_logger(), "Retrying navigation goal (%zu/%d)",
			goal_failures_, max_goal_retries_);
		if (last_goal_is_weed_ && last_navigation_goal_valid_) {
			set_state(State::Searching);
			send_navigation_goal(last_navigation_goal_, true, last_weed_id_, last_weed_name_);
			return;
		}
		queue_request(GoalRequest::RETRY_CURRENT_GOAL);
		return;
	}
	RCLCPP_ERROR(get_logger(), "Navigation goal is unreachable after %zu retries", goal_failures_ - 1);
	//request_next_goal(true);
	if (last_goal_is_weed_) {
		queue_request(GoalRequest::WEED_UNREACHABLE, last_weed_id_);
	} else {
		queue_request(GoalRequest::SKIP_CURRENT_GOAL);
	}
}

bool WombatNode::mission_stopped() const
{
	return state_ == State::Complete || state_ == State::Abort;
}

void WombatNode::abort_mission()
{
	// Reuse cancellation and pending-request cleanup without reporting success.
	complete_mission();

	last_navigation_goal_valid_ = false;
	set_state(State::Abort);

	RCLCPP_ERROR(
		get_logger(), "Mission aborted: Nav2 unavailable for %.1f seconds",
		nav_unavailable_timeout_);
}

void WombatNode::handle_override(bool enabled)
{
	if (enabled) {
		cancel_active_goal();
	} else {
		request_next_goal(GoalRequest::REQUEST_NEXT_GOAL);
	}
}

void WombatNode::handle_estop()
{
	cancel_active_goal();
}

void WombatNode::handle_resume()
{
	request_next_goal(GoalRequest::REQUEST_NEXT_GOAL);
}

void WombatNode::cancel_active_goal()
{
	if (active_goal_) {
		navigation_client_->async_cancel_goal(active_goal_);
		active_goal_.reset();
	}
	mission_request_in_flight_ = false;
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<WombatNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
