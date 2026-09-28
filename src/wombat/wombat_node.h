#ifndef RS1_WOMBAT_NODE_H_
#define RS1_WOMBAT_NODE_H_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rs1/srv/wombat_next_goal.hpp"

#include <memory>
#include <string>

class WombatNode : public rclcpp::Node
{
public:
	WombatNode();

private:
	void request_next_goal();
	void handle_goal_response(rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future);
	void control_timer_callback();
	bool current_pose(geometry_msgs::msg::PoseStamped & pose);
	void publish_cmd_vel(double linear_speed, double angular_speed);
	static double normalize_yaw(double yaw);
	static double quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation);

	const std::string global_frame_ = "map";
	const std::string robot_base_frame_ = "base_link";
	const double linear_speed_ = 0.5;
	const double angular_speed_ = 0.8;
	const double goal_tolerance_ = 0.2;
	const double turn_tolerance_ = 0.08;

	bool has_goal_{false};
	geometry_msgs::msg::PoseStamped current_goal_;

	rclcpp::Client<rs1::srv::WombatNextGoal>::SharedPtr goal_client_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::TimerBase::SharedPtr control_timer_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif