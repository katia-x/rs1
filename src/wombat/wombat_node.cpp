#include "wombat_node.h"

using namespace std::chrono_literals;

// Create the Mission service client and Nav2 action client.
WombatNode::WombatNode() : Node("wombat_node"),
	global_frame_(declare_parameter("global_frame", "map"))
{
	mission_client_ = create_client<rs1::srv::WombatNextGoal>("wombat_next_goal");
	navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
		this, declare_parameter("navigation_action", "/husky1/navigate_to_pose"));
	mission_retry_timer_ = create_wall_timer(1s, std::bind(&WombatNode::request_next_goal, this));
	request_next_goal();
}

// Ask Mission for the next search or weed goal.
void WombatNode::request_next_goal()
{
	if (active_goal_ || mission_request_in_flight_) {
		return;
	}

	if (!mission_client_->wait_for_service(1s)) {
		RCLCPP_WARN(get_logger(), "Mission service is not available yet");
		return;
	}

	mission_request_in_flight_ = true;
	auto request = std::make_shared<rs1::srv::WombatNextGoal::Request>();
	auto future = mission_client_->async_send_request(request, std::bind(&WombatNode::handle_mission_goal, this, std::placeholders::_1));
	(void)future;
}

// Forward Mission's goal to Nav2, which handles path planning and control.
void WombatNode::handle_mission_goal(rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future)
{
	std::shared_ptr<rs1::srv::WombatNextGoal::Response> response;
	try {
		response = future.get();
	} catch (const std::exception & error) {
		mission_request_in_flight_ = false;
		RCLCPP_WARN(get_logger(), "Mission service call failed: %s", error.what());
		request_next_goal();
		return;
	}
	mission_request_in_flight_ = false;

	if (!response->success) {
		request_next_goal();
		return;
	}

	response->goal.header.frame_id = response->goal.header.frame_id.empty() ? global_frame_ : response->goal.header.frame_id;
	response->goal.header.stamp = now();

	if (!navigation_client_->wait_for_action_server(2s)) {
		RCLCPP_WARN(get_logger(), "Nav2 NavigateToPose action is not available yet");
		return;
	}

	NavigateToPose::Goal navigation_goal;
	navigation_goal.pose = response->goal;
	rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
	options.goal_response_callback = std::bind(
		&WombatNode::handle_nav_goal_response, this, std::placeholders::_1);
	options.result_callback = std::bind(&WombatNode::handle_nav_result, this, std::placeholders::_1);
	(void)navigation_client_->async_send_goal(navigation_goal, options);
}

void WombatNode::handle_nav_goal_response(NavigateGoalHandle::SharedPtr goal_handle)
{
	if (!goal_handle) {
		RCLCPP_ERROR(get_logger(), "Nav2 rejected the goal");
		request_next_goal();
		return;
	}

	active_goal_ = goal_handle;
	RCLCPP_INFO(get_logger(), "Nav2 accepted the goal");
}

void WombatNode::handle_nav_result(const NavigateGoalHandle::WrappedResult & result)
{
	active_goal_.reset();
	if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_INFO(get_logger(), "Nav2 reached the goal");
	} else {
		RCLCPP_WARN(get_logger(), "Nav2 failed to reach the goal; requesting another goal");
	}
	request_next_goal();
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<WombatNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
