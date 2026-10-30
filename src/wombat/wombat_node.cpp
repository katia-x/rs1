#include "wombat_node.h"

#include <cmath>
#include <stdexcept>

using namespace std::chrono_literals;

WombatNode::WombatNode()
: Node("wombat_node"),
  global_frame_(declare_parameter("global_frame", "map")),
  lane_start_wait_duration_(declare_parameter("lane_start_wait_duration", 2.0))
{
	if (!std::isfinite(lane_start_wait_duration_) || lane_start_wait_duration_ < 0.0) {
		throw std::invalid_argument("lane_start_wait_duration must be finite and non-negative");
	}
	mission_client_ = create_client<rs1::srv::WombatNextGoal>("wombat_next_goal");
	navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
		this, declare_parameter("navigation_action", "/husky1/navigate_to_pose"));
	mission_retry_timer_ = create_wall_timer(1s, [this]() {
		timer_callback();
	});
}

void WombatNode::timer_callback()
{
	if (active_goal_ || mission_request_in_flight_ || cancel_in_flight_) {
		if (active_goal_ && (state_ == State::Searching || state_ == State::GoingToWeed)) {
			cancel_for_reassessment();
		}
		return;
	}
	if (state_ == State::PreLaneWait) {
		if (std::chrono::steady_clock::now() < lane_wait_until_) { return; }
		state_ = State::Searching;
	}
	if (!request_pending_ && (state_ == State::WaitingForRegion || state_ == State::Complete)) {
		request_pending_ = true;
	}
	if (!request_pending_ && (state_ == State::Searching || state_ == State::GoingToWeed)) {
		queue_request(false, true);
	}
	if (!request_pending_ && state_ == State::Recovery) {
		queue_request(true, false);
	}
	if (!request_pending_) { return; }
	request_next_goal(pending_skip_current_goal_, pending_reassess_, pending_weed_handled_);
}

void WombatNode::queue_request(bool skip_current_goal, bool reassess, bool weed_handled)
{
	request_pending_ = true;
	pending_skip_current_goal_ = skip_current_goal;
	pending_reassess_ = reassess;
	pending_weed_handled_ = weed_handled;
}

void WombatNode::request_next_goal(bool skip_current_goal, bool reassess, bool weed_handled)
{
	if (active_goal_ || mission_request_in_flight_ || cancel_in_flight_) { return; }

	if (!mission_client_->wait_for_service(1s)) {
		RCLCPP_WARN(get_logger(), "Mission service is not available yet");
		return;
	}

	request_pending_ = false;
	mission_request_in_flight_ = true;
	auto request = std::make_shared<rs1::srv::WombatNextGoal::Request>();
	request->skip_current_goal = skip_current_goal;
	request->reassess_current_goal = reassess;
	request->weed_handled = weed_handled;
	request->handled_weed_id = last_weed_id_;
	mission_client_->async_send_request(
		request, std::bind(&WombatNode::handle_mission_goal, this, std::placeholders::_1));
}

void WombatNode::handle_mission_goal(rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future)
{
	try {
		auto response = future.get();
		mission_request_in_flight_ = false;
		if (response->mission_complete) {
			state_ = State::Complete;
			return;
		}

		if (!response->success) {
			state_ = State::WaitingForRegion;
			request_pending_ = true;
			return;
		}

		response->goal.header.frame_id = response->goal.header.frame_id.empty()
			? global_frame_ : response->goal.header.frame_id;
		response->goal.header.stamp = now();

		const double repeated_goal_distance = std::hypot(
			response->goal.pose.position.x - last_goal_.pose.position.x,
			response->goal.pose.position.y - last_goal_.pose.position.y);
		if (!response->is_turn && last_goal_valid_ && repeated_goal_distance < 0.1) {
			RCLCPP_WARN(
				get_logger(),
				"Waypoint (%.2f, %.2f) repeated without progress; skipping it",
				response->goal.pose.position.x, response->goal.pose.position.y);
			state_ = State::Recovery;
			queue_request(true, false);
			return;
		}

		if (!navigation_client_->wait_for_action_server(2s)) {
			RCLCPP_WARN(get_logger(), "Nav2 NavigateToPose action is not available yet");
			state_ = State::Recovery;
			queue_request(false, false);
			return;
		}

		NavigateToPose::Goal goal;
		goal.pose = response->goal;
		last_goal_ = response->goal;
		last_goal_valid_ = true;
		last_goal_is_weed_ = response->is_weed;
		if (response->is_lane_start) {
			state_ = State::ApproachingStart;
		} else if (response->is_turn) {
			state_ = State::Turning;
		} else if (response->is_weed) {
			state_ = State::GoingToWeed;
		} else {
			state_ = State::Searching;
		}

		if (last_goal_is_weed_) {
			last_weed_id_ = response->weed_id;
		}

		rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
		options.goal_response_callback = std::bind(
			&WombatNode::handle_nav_goal_response, this, std::placeholders::_1);
		options.result_callback = std::bind(
			&WombatNode::handle_nav_result, this, std::placeholders::_1);
		navigation_client_->async_send_goal(goal, options);
	} catch (const std::exception & error) {
		mission_request_in_flight_ = false;
		RCLCPP_WARN(get_logger(), "Mission service call failed: %s", error.what());
		state_ = State::Recovery;
		queue_request(false, false);
	}
}

void WombatNode::handle_nav_goal_response(NavigateGoalHandle::SharedPtr goal_handle)
{
	if (!goal_handle) {
		RCLCPP_ERROR(get_logger(), "Nav2 rejected the goal; skipping it");
		state_ = State::Recovery;
		queue_request(true, false);
		return;
	}

	active_goal_ = goal_handle;
	RCLCPP_INFO(get_logger(), "Nav2 accepted the goal");
}

void WombatNode::handle_nav_result(const NavigateGoalHandle::WrappedResult & result)
{
	active_goal_.reset();
	if (intentional_cancel_) {
		intentional_cancel_ = false;
		cancel_in_flight_ = false;
		queue_request(false, true);
		return;
	}
	if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_INFO(get_logger(), "Nav2 reached the goal");
		if (last_goal_is_weed_) {
			queue_request(false, false, true);
			state_ = State::Searching;
		} else if (state_ == State::ApproachingStart || state_ == State::Turning) {
			state_ = State::PreLaneWait;
			lane_wait_until_ = std::chrono::steady_clock::now() +
				std::chrono::duration_cast<std::chrono::steady_clock::duration>(
					std::chrono::duration<double>(lane_start_wait_duration_));
		} else {
			queue_request(false, true);
		}
		return;
	}

	RCLCPP_WARN(get_logger(), "Nav2 failed to reach the goal; skipping it");
	state_ = State::Recovery;
	queue_request(true, false);
}

void WombatNode::cancel_for_reassessment()
{
	if (!active_goal_ || cancel_in_flight_) { return; }
	cancel_in_flight_ = true;
	intentional_cancel_ = true;
	navigation_client_->async_cancel_goal(active_goal_,
		[this](auto future) {
			try {
				if (future.get()->return_code != action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
					RCLCPP_WARN(get_logger(), "Nav2 goal cancellation was rejected during reassessment");
				}
			} catch (const std::exception & error) {
				RCLCPP_WARN(get_logger(), "Nav2 goal cancellation failed: %s", error.what());
			}
		});
}
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<WombatNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
