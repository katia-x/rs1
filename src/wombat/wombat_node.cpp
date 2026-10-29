#include "wombat_node.h"

#include <cmath>
#include <functional>
#include <stdexcept>

using namespace std::chrono_literals;

WombatNode::WombatNode()
: Node("wombat_node"),
  global_frame_(declare_parameter("global_frame", "map"))
{
	const auto override_command_topic = declare_parameter<std::string>(
		"manual_override_command_topic", "/husky1/manual_override_command");
	const auto override_state_topic = declare_parameter<std::string>(
		"manual_override_state_topic", "/husky1/manual_override");
	mission_client_ = create_client<rs1::srv::WombatNextGoal>("wombat_next_goal");
	navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
		this, declare_parameter("navigation_action", "/husky1/navigate_to_pose"));
	override_state_publisher_ = create_publisher<std_msgs::msg::Bool>(
		override_state_topic, rclcpp::QoS(1).transient_local());
	override_subscription_ = create_subscription<std_msgs::msg::Bool>(
		override_command_topic, rclcpp::QoS(10),
		std::bind(&WombatNode::override_callback, this, std::placeholders::_1));
	publish_override_state();
	mission_retry_timer_ = create_wall_timer(1s, [this]() { request_next_goal(); });
	request_next_goal();
}

void WombatNode::request_next_goal(bool skip_current_goal)
{
	if (manual_override_ || active_goal_ || mission_request_in_flight_) {
		return;
	}

	if (!mission_client_->wait_for_service(1s)) {
		RCLCPP_WARN(get_logger(), "Mission service is not available yet");
		return;
	}

	mission_request_in_flight_ = true;
	auto request = std::make_shared<rs1::srv::WombatNextGoal::Request>();
	request->skip_current_goal = skip_current_goal;
	mission_client_->async_send_request(
		request, std::bind(&WombatNode::handle_mission_goal, this, std::placeholders::_1));
}

void WombatNode::handle_mission_goal(rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future)
{
	try {
		auto response = future.get();
		mission_request_in_flight_ = false;

		if (!response->success) {
			request_next_goal();
			return;
		}
		if (manual_override_) {
			return;
		}

		response->goal.header.frame_id = response->goal.header.frame_id.empty()
			? global_frame_ : response->goal.header.frame_id;
		response->goal.header.stamp = now();

		const double repeated_goal_distance = std::hypot(
			response->goal.pose.position.x - last_goal_.pose.position.x,
			response->goal.pose.position.y - last_goal_.pose.position.y);
		if (last_goal_valid_ && repeated_goal_distance < 0.1) {
			RCLCPP_WARN(
				get_logger(),
				"Waypoint (%.2f, %.2f) repeated without progress; skipping it",
				response->goal.pose.position.x, response->goal.pose.position.y);
			request_next_goal(true);
			return;
		}

		if (!navigation_client_->wait_for_action_server(2s)) {
			RCLCPP_WARN(get_logger(), "Nav2 NavigateToPose action is not available yet");
			return;
		}

		NavigateToPose::Goal goal;
		goal.pose = response->goal;
		last_goal_ = response->goal;
		last_goal_valid_ = true;

		rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
		options.goal_response_callback = std::bind(
			&WombatNode::handle_nav_goal_response, this, std::placeholders::_1);
		options.result_callback = std::bind(
			&WombatNode::handle_nav_result, this, std::placeholders::_1);
		navigation_client_->async_send_goal(goal, options);
	} catch (const std::exception & error) {
		mission_request_in_flight_ = false;
		RCLCPP_WARN(get_logger(), "Mission service call failed: %s", error.what());
		request_next_goal();
	}
}

void WombatNode::handle_nav_goal_response(NavigateGoalHandle::SharedPtr goal_handle)
{
	if (!goal_handle) {
		RCLCPP_ERROR(get_logger(), "Nav2 rejected the goal; skipping it");
		request_next_goal(true);
		return;
	}
	if (manual_override_) {
		navigation_client_->async_cancel_goal(goal_handle);
		return;
	}

	active_goal_ = goal_handle;
	RCLCPP_INFO(get_logger(), "Nav2 accepted the goal");
}

void WombatNode::handle_nav_result(const NavigateGoalHandle::WrappedResult & result)
{
	active_goal_.reset();
	if (manual_override_) {
		return;
	}
	if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_INFO(get_logger(), "Nav2 reached the goal");
		return;
	}

	RCLCPP_WARN(get_logger(), "Nav2 failed to reach the goal; skipping it");
	request_next_goal(true);
}

void WombatNode::override_callback(const std_msgs::msg::Bool::ConstSharedPtr message)
{
	if (!message || message->data == manual_override_) {
		return;
	}
	manual_override_ = message->data;
	if (manual_override_) {
		cancel_active_goal();
		RCLCPP_WARN(get_logger(),
			"Manual override enabled; autonomous navigation stopped; Teleop may drive /husky1/cmd_vel");
	} else {
		RCLCPP_INFO(get_logger(), "Manual override disabled; autonomous navigation may resume");
		request_next_goal();
	}
	publish_override_state();
}

void WombatNode::publish_override_state()
{
	std_msgs::msg::Bool state;
	state.data = manual_override_;
	override_state_publisher_->publish(state);
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
