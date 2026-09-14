#ifndef RS1_WOMBAT_NODE_H_
#define RS1_WOMBAT_NODE_H_

#include "wombat/wombat_processing.h"

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

class WombatNode : public rclcpp::Node
{
public:
	WombatNode();

private:
	// Nav2 exposes NavigateToPose as an action, so the node receives a result
	// before choosing the next search or weed goal.
	using NavigateToPose = nav2_msgs::action::NavigateToPose;
	using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

	// Receive the latest weed-location array without changing plant perception.
	void weed_callback(const geometry_msgs::msg::PoseArray::SharedPtr message);
	// Periodically select and submit one goal while the node is idle.
	void control_timer_callback();
	// Read the robot pose from the global frame to its base frame.
	bool current_pose(geometry_msgs::msg::PoseStamped & pose);
	// Convert a planner goal into a Nav2 NavigateToPose action request.
	void send_goal(const WombatProcessing::Goal & goal);
	// Handle acceptance or rejection by the Nav2 action server.
	void goal_response_callback(const GoalHandle::SharedPtr & goal_handle);
	// Handle navigation completion and start weed inspection when required.
	void result_callback(const GoalHandle::WrappedResult & result);
	// Finish the configurable pause and mark the inspected weed as handled.
	void finish_inspection();

	std::string global_frame_;
	std::string robot_base_frame_;
	double inspection_pause_seconds_;
	// These flags prevent competing goals while Nav2 is moving or the robot is
	// paused at a weed.
	bool goal_active_{false};
	bool inspecting_weed_{false};
	WombatProcessing::Goal active_goal_;

	WombatProcessing processing_;
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr weed_subscription_;
	rclcpp::TimerBase::SharedPtr control_timer_;
	rclcpp::TimerBase::SharedPtr inspection_timer_;
	rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif