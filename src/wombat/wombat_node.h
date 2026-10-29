#ifndef RS1_WOMBAT_NODE_H_
#define RS1_WOMBAT_NODE_H_

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include "rs1/srv/wombat_next_goal.hpp"

#include <memory>
#include <string>

class WombatNode : public rclcpp::Node
{
public:
	WombatNode();

private:
	void request_next_goal(bool skip_current_goal = false);
	void handle_mission_goal(rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future);
	void handle_nav_goal_response(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
	void handle_nav_result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
	void override_callback(const std_msgs::msg::Bool::ConstSharedPtr message);
	void publish_override_state();
	void cancel_active_goal();

	const std::string global_frame_;

	using NavigateToPose = nav2_msgs::action::NavigateToPose;
	using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

	rclcpp::Client<rs1::srv::WombatNextGoal>::SharedPtr mission_client_;
	rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
	NavigateGoalHandle::SharedPtr active_goal_;
	geometry_msgs::msg::PoseStamped last_goal_;
	bool last_goal_valid_{false};
	rclcpp::TimerBase::SharedPtr mission_retry_timer_;
	bool mission_request_in_flight_{false};
	bool manual_override_{false};
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr override_subscription_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr override_state_publisher_;
};

#endif