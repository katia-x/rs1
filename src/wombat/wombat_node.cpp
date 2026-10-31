#include "wombat_node.h"

#include <cmath>
#include <stdexcept>

using namespace std::chrono_literals;

WombatNode::WombatNode()
: Node("wombat_node"),
  global_frame_(declare_parameter("global_frame", "map")),
	navigation_behavior_tree_(declare_parameter("navigation_behavior_tree", "")),
  lane_start_wait_duration_(declare_parameter("lane_start_wait_duration", 2.0)),
	reassessment_frequency_(declare_parameter("reassessment_frequency", 1.0)),
	max_goal_retries_(declare_parameter("max_goal_retries", 3))
{
	if (!std::isfinite(lane_start_wait_duration_) || lane_start_wait_duration_ < 0.0) {
		throw std::invalid_argument("lane_start_wait_duration must be finite and non-negative");
	}
	if (!std::isfinite(reassessment_frequency_) || reassessment_frequency_ <= 0.0) {
		throw std::invalid_argument("reassessment_frequency must be finite and positive");
	}
	if (max_goal_retries_ < 0) {
		throw std::invalid_argument("max_goal_retries must be non-negative");
	}
	mission_client_ = create_client<rs1::srv::WombatNextGoal>("wombat_next_goal");
	navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
		this, declare_parameter("navigation_action", "/husky1/navigate_to_pose"));
	auto reassessment_period = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::duration<double>(1.0 / reassessment_frequency_));
	if (reassessment_period < 1ms) { reassessment_period = 1ms; }
	mission_retry_timer_ = create_wall_timer(reassessment_period, [this]() {
		timer_callback();
	});
}

void WombatNode::timer_callback()
{
	if (mission_request_in_flight_ || nav_goal_request_in_flight_) {
		return;
	}
	if (active_goal_) {
		if (state_ == State::Searching && !last_goal_is_weed_) {
			request_next_goal(false, true);
		}
		return;
	}
	if (state_ == State::PreLaneWait) {
		if (std::chrono::steady_clock::now() < lane_wait_until_) { return; }
		state_ = State::Searching;
	}
	if (state_ == State::Complete) {
		return;
	}
	if (!pending_request_.pending && state_ == State::WaitingForRegion) {
		pending_request_.pending = true;
	}
	if (!pending_request_.pending && state_ == State::Searching) {
		queue_request(false, false);
	}
	if (!pending_request_.pending && state_ == State::Recovery) {
		queue_request(true, false);
	}
	if (!pending_request_.pending) { return; }
	request_next_goal(
		pending_request_.skip_current_goal,
		pending_request_.reassess,
		pending_request_.weed_handled,
		pending_request_.retry_current_goal,
		pending_request_.weed_unreachable,
		pending_request_.unreachable_weed_id);
}

void WombatNode::queue_request(
	bool skip_current_goal, bool reassess, bool weed_handled,
	bool retry_current_goal, bool weed_unreachable, std::uint64_t unreachable_weed_id)
{
	pending_request_.pending = true;
	pending_request_.skip_current_goal = skip_current_goal;
	pending_request_.retry_current_goal = retry_current_goal;
	pending_request_.reassess = reassess;
	pending_request_.weed_handled = weed_handled;
	pending_request_.weed_unreachable = weed_unreachable;
	pending_request_.unreachable_weed_id = unreachable_weed_id;
}

void WombatNode::request_next_goal(
	bool skip_current_goal, bool reassess, bool weed_handled,
	bool retry_current_goal, bool weed_unreachable, std::uint64_t unreachable_weed_id)
{
	if ((active_goal_ && !reassess) || mission_request_in_flight_ || nav_goal_request_in_flight_) {
		return;
	}

	if (!mission_client_->wait_for_service(1s)) {
		RCLCPP_WARN(get_logger(), "Mission service is not available yet");
		return;
	}

	pending_request_.pending = false;
	mission_request_in_flight_ = true;
	mission_request_retry_ = retry_current_goal;
	mission_request_reassessment_ = reassess;
	auto request = std::make_shared<rs1::srv::WombatNextGoal::Request>();
	request->skip_current_goal = skip_current_goal;
	request->retry_current_goal = retry_current_goal;
	request->reassess_current_goal = reassess;
	request->weed_handled = weed_handled;
	request->handled_weed_id = last_weed_id_;
	request->weed_unreachable = weed_unreachable;
	request->unreachable_weed_id = unreachable_weed_id;
	mission_client_->async_send_request(
		request, std::bind(&WombatNode::handle_mission_goal, this, std::placeholders::_1));
}

void WombatNode::handle_mission_goal(rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future)
{
	try {
		auto response = future.get();
		mission_request_in_flight_ = false;
		const bool retry_request = mission_request_retry_;
		const bool reassessment_request = mission_request_reassessment_;
		mission_request_retry_ = false;
		mission_request_reassessment_ = false;
		if (!retry_request && !reassessment_request) { goal_failures_ = 0; }
		if (response->mission_complete) {
			RCLCPP_INFO(get_logger(), "Mission complete: no further navigation goals remain");
			state_ = State::Complete;
			return;
		}

		if (!response->success) {
			state_ = State::WaitingForRegion;
			pending_request_.pending = true;
			return;
		}

		response->goal.header.frame_id = response->goal.header.frame_id.empty()
			? global_frame_ : response->goal.header.frame_id;
		response->goal.header.stamp = now();

		last_goal_is_weed_ = response->is_weed;
		if (last_goal_is_weed_) { last_weed_id_ = response->weed_id; }
		if (response->is_lane_start) {
			state_ = State::ApproachingStart;
		} else if (response->is_turn) {
			state_ = State::Turning;
		} else {
			state_ = State::Searching;
		}

		if (!navigation_client_->wait_for_action_server(2s)) {
			RCLCPP_WARN(get_logger(), "Nav2 NavigateToPose action is not available yet");
			state_ = State::Recovery;
			handle_nav_failure();
			return;
		}

		NavigateToPose::Goal goal;
		goal.pose = response->goal;
		goal.behavior_tree = navigation_behavior_tree_;

		rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
		options.goal_response_callback = std::bind(
			&WombatNode::handle_nav_goal_response, this, std::placeholders::_1);
		options.result_callback = std::bind(
			&WombatNode::handle_nav_result, this, std::placeholders::_1);
		nav_goal_request_in_flight_ = true;
		navigation_client_->async_send_goal(goal, options);
	} catch (const std::exception & error) {
		mission_request_in_flight_ = false;
		nav_goal_request_in_flight_ = false;
		RCLCPP_WARN(get_logger(), "Mission service call failed: %s", error.what());
		state_ = State::Recovery;
		queue_request(false, false);
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
	if (mission_request_in_flight_ || nav_goal_request_in_flight_ ||
		!active_goal_ || result.goal_id != active_goal_->get_goal_id()) {
		return;
	}
	active_goal_.reset();
	if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_INFO(get_logger(), "Nav2 reached the goal");
		goal_failures_ = 0;
		if (last_goal_is_weed_) {
			queue_request(false, false, true);
			state_ = State::Searching;
		} else if (state_ == State::ApproachingStart || state_ == State::Turning) {
			state_ = State::PreLaneWait;
			lane_wait_until_ = std::chrono::steady_clock::now() +
				std::chrono::duration_cast<std::chrono::steady_clock::duration>(
					std::chrono::duration<double>(lane_start_wait_duration_));
		} else {
			queue_request(false, false);
		}
		return;
	}

	RCLCPP_WARN(get_logger(), "Nav2 failed to reach the goal");
	state_ = State::Recovery;
	handle_nav_failure();
}

void WombatNode::handle_nav_failure()
{
	++goal_failures_;
	if (goal_failures_ <= static_cast<std::size_t>(max_goal_retries_)) {
		RCLCPP_WARN(get_logger(), "Retrying navigation goal (%zu/%d)",
			goal_failures_, max_goal_retries_);
		queue_request(false, false, false, true);
		return;
	}

	RCLCPP_ERROR(get_logger(), "Navigation goal is unreachable after %zu retries", goal_failures_ - 1);
	if (last_goal_is_weed_) {
		queue_request(true, false, false, false, true, last_weed_id_);
	} else {
		queue_request(true, false);
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
