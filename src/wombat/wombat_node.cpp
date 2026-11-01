#include "wombat_node.h"

#include <cmath>
#include <stdexcept>

using namespace std::chrono_literals;
using GoalRequest = rs1::srv::WombatNextGoal::Request;

WombatNode::WombatNode()
: Node("wombat_node"),
  global_frame_(declare_parameter("global_frame", "map")),
  lane_start_wait_duration_(declare_parameter("lane_start_wait_duration", 2.0)),
  max_goal_retries_(declare_parameter("max_goal_retries", 3))
{
	if (!std::isfinite(lane_start_wait_duration_) || lane_start_wait_duration_ < 0.0) {
		throw std::invalid_argument("lane_start_wait_duration must be finite and non-negative");
	}
	if (max_goal_retries_ < 0) {
		throw std::invalid_argument("max_goal_retries must be non-negative");
	}
	mission_client_ = create_client<rs1::srv::WombatNextGoal>("wombat_next_goal");
	weed_goal_subscription_ = create_subscription<rs1::msg::WombatWeedGoal>(
		"/wombat/weed_goal", rclcpp::QoS(1).transient_local(),
		std::bind(&WombatNode::weed_goal_callback, this, std::placeholders::_1));
	status_subscription_ = create_subscription<rs1::msg::WombatStatus>(
		"/wombat/status", rclcpp::QoS(1).transient_local(),
		std::bind(&WombatNode::status_callback, this, std::placeholders::_1));
	navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
		this, declare_parameter("navigation_action", "/husky1/navigate_to_pose"));
	mission_retry_timer_ = create_wall_timer(100ms, [this]() { timer_callback(); });
}

void WombatNode::timer_callback()
{
	if (mission_request_in_flight_ || nav_goal_request_in_flight_ || nav_cancel_in_flight_) { return; }
	if (pending_weed_goal_.pending) {
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
		return;
	case State::PreLaneWait:
		if (std::chrono::steady_clock::now() < lane_wait_until_) { return; }
		state_ = State::Searching;
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
	if (!pending_request_.pending) { return; }
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
	if (active_goal_ || mission_request_in_flight_ || nav_goal_request_in_flight_) { return; }
	if (!mission_client_->wait_for_service(1s)) {
		RCLCPP_WARN(get_logger(), "Mission service is not available yet");
		return;
	}
	pending_request_.pending = false;
	mission_request_in_flight_ = true;
	mission_request_operation_ = operation;
	mission_request_generation_ = mission_generation_;
	auto request = std::make_shared<rs1::srv::WombatNextGoal::Request>();
	request->operation = operation;
	request->weed_id = weed_id;
	mission_client_->async_send_request(
		request, std::bind(&WombatNode::handle_mission_goal, this, std::placeholders::_1));
}

void WombatNode::weed_goal_callback(const rs1::msg::WombatWeedGoal::SharedPtr message)
{
	if (state_ == State::Complete || message->mission_generation < mission_generation_) {
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
	if (state_ == State::Complete || !pending_weed_goal_.pending || active_goal_ || nav_cancel_in_flight_) {
		return;
	}
	const auto weed_goal = pending_weed_goal_.goal;
	pending_weed_goal_.pending = false;
	state_ = State::Searching;
	RCLCPP_INFO(get_logger(), "Sending weed goal %llu to Nav2",
		static_cast<unsigned long long>(weed_goal.weed_id));
	send_navigation_goal(weed_goal.goal, true, weed_goal.weed_id);
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
	canceling_goal_.reset();
	nav_cancel_in_flight_ = false;
	state_ = State::Complete;
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
	if (message->mission_complete) {
		complete_mission();
	}
}

void WombatNode::reset_for_new_region()
{
	if (active_goal_) {
		navigation_client_->async_cancel_goal(active_goal_);
		active_goal_.reset();
	}
	canceling_goal_.reset();
	nav_cancel_in_flight_ = false;
	pending_weed_goal_.pending = false;
	pending_request_ = PendingRequest{};
	mission_request_operation_ = rs1::srv::WombatNextGoal::Request::REQUEST_NEXT_GOAL;
	goal_failures_ = 0;
	last_goal_is_weed_ = false;
	last_weed_id_ = 0;
	last_navigation_goal_valid_ = false;
	state_ = State::WaitingForRegion;
}

void WombatNode::send_navigation_goal(
	const geometry_msgs::msg::PoseStamped & pose, bool is_weed, std::uint64_t weed_id)
{
	if (!navigation_client_->wait_for_action_server(2s)) {
		RCLCPP_WARN(get_logger(), "Nav2 NavigateToPose action is not available yet");
		state_ = State::Recovery;
		handle_nav_failure();
		return;
	}
	NavigateToPose::Goal goal;
	goal.pose = pose;
	goal.pose.header.frame_id = goal.pose.header.frame_id.empty() ? global_frame_ : goal.pose.header.frame_id;
	goal.pose.header.stamp = now();
	last_goal_is_weed_ = is_weed;
	if (is_weed) { last_weed_id_ = weed_id; }
	last_navigation_goal_ = goal.pose;
	last_navigation_goal_valid_ = true;
	rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
	const auto goal_generation = mission_generation_;
	options.goal_response_callback =
		[this, goal_generation](NavigateGoalHandle::SharedPtr goal_handle) {
			if (goal_generation != mission_generation_ || state_ == State::Complete) {
				nav_goal_request_in_flight_ = false;
				if (goal_handle) { navigation_client_->async_cancel_goal(goal_handle); }
				return;
			}
			handle_nav_goal_response(goal_handle);
		};
	options.result_callback =
		[this, goal_generation](const NavigateGoalHandle::WrappedResult & result) {
			if (goal_generation == mission_generation_ && state_ != State::Complete) {
				handle_nav_result(result);
			}
		};
	nav_goal_request_in_flight_ = true;
	navigation_client_->async_send_goal(goal, options);
}

void WombatNode::handle_mission_goal(
	rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future)
{
	try {
		auto response = future.get();
		mission_request_in_flight_ = false;
		if (mission_request_generation_ != mission_generation_) { return; }
		if (response->mission_complete) {
			complete_mission();
			return;
		}
		if (!response->success) {
			state_ = State::WaitingForRegion;
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
			state_ = State::ApproachingStart;
		} else if (response->is_turn) {
			state_ = State::Turning;
		} else {
			state_ = State::Searching;
		}
		send_navigation_goal(response->goal, response->is_weed, response->weed_id);
	} catch (const std::exception & error) {
		mission_request_in_flight_ = false;
		RCLCPP_WARN(get_logger(), "Mission service call failed: %s", error.what());
		state_ = State::Recovery;
		queue_request(mission_request_operation_, last_weed_id_);
	}
}

void WombatNode::handle_nav_goal_response(NavigateGoalHandle::SharedPtr goal_handle)
{
	nav_goal_request_in_flight_ = false;
	if (!goal_handle) {
		RCLCPP_ERROR(get_logger(), "Nav2 rejected the goal");
		state_ = State::Recovery;
		handle_nav_failure();
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
		if (state_ != State::Complete) { dispatch_pending_weed_goal(); }
		return;
	}
	if (nav_cancel_in_flight_) { return; }
	if (nav_goal_request_in_flight_ || !active_goal_ ||
		result.goal_id != active_goal_->get_goal_id()) { return; }
	active_goal_.reset();
	if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_INFO(get_logger(), "Nav2 reached the goal");
		goal_failures_ = 0;
		if (last_goal_is_weed_) {
			queue_request(GoalRequest::WEED_HANDLED, last_weed_id_);
			state_ = State::Searching;
		} else if (state_ == State::ApproachingStart || state_ == State::Turning) {
			state_ = State::PreLaneWait;
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
	state_ = State::Recovery;
	handle_nav_failure();
}

void WombatNode::handle_nav_failure()
{
	++goal_failures_;
	if (goal_failures_ <= static_cast<std::size_t>(max_goal_retries_)) {
		RCLCPP_WARN(get_logger(), "Retrying navigation goal (%zu/%d)",
			goal_failures_, max_goal_retries_);
		if (last_goal_is_weed_ && last_navigation_goal_valid_) {
			state_ = State::Searching;
			send_navigation_goal(last_navigation_goal_, true, last_weed_id_);
			return;
		}
		queue_request(GoalRequest::RETRY_CURRENT_GOAL);
		return;
	}
	RCLCPP_ERROR(get_logger(), "Navigation goal is unreachable after %zu retries", goal_failures_ - 1);
	if (last_goal_is_weed_) {
		queue_request(GoalRequest::WEED_UNREACHABLE, last_weed_id_);
	} else {
		queue_request(GoalRequest::SKIP_CURRENT_GOAL);
	}
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<WombatNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
