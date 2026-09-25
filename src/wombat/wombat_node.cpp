#include "wombat/wombat_node.h"

#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

using namespace std::chrono_literals;

WombatNode::WombatNode()
: Node("wombat_node")
{
	global_frame_ = declare_parameter("global_frame", "map");
	robot_base_frame_ = declare_parameter("robot_base_frame", "base_link");
	linear_speed_ = declare_parameter("linear_speed", 0.5);
	angular_speed_ = declare_parameter("angular_speed", 0.8);
	goal_tolerance_ = declare_parameter("goal_tolerance", 0.25);
	turn_tolerance_ = declare_parameter("turn_tolerance", 0.08);

	goal_client_ = create_client<rs1::srv::WombatNextGoal>("wombat_next_goal");
	cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	control_timer_ = create_wall_timer(50ms, std::bind(&WombatNode::control_timer_callback, this));

	RCLCPP_INFO(get_logger(), "Wombat requesting goals from the mission service");
	request_next_goal();
}

void WombatNode::request_next_goal()
{
	if (!goal_client_->wait_for_service(1s)) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Waiting for mission wombat_next_goal service");
		return;
	}

	auto request = std::make_shared<rs1::srv::WombatNextGoal::Request>();
	auto future = goal_client_->async_send_request(
		request, std::bind(&WombatNode::handle_goal_response, this, std::placeholders::_1));
	(void)future;
}

void WombatNode::handle_goal_response(rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future)
{
	try {
		auto response = future.get();
		if (!response) {
			RCLCPP_WARN(get_logger(), "Mission did not return a valid next goal");
			return;
		}

		current_goal_ = response->goal;
		current_goal_.header.frame_id = current_goal_.header.frame_id.empty() ? global_frame_ : current_goal_.header.frame_id;
		has_goal_ = true;
		moving_ = true;
		RCLCPP_INFO(get_logger(), "Received goal at x=%f y=%f is_weed=%s plant=%s",
			current_goal_.pose.position.x,
			current_goal_.pose.position.y,
			response->is_weed ? "true" : "false",
			response->plant_type.c_str());
	} catch (const std::exception & ex) {
		RCLCPP_ERROR(get_logger(), "Failed to receive next goal: %s", ex.what());
	}
}

void WombatNode::control_timer_callback()
{
	if (!moving_ || !has_goal_) {
		return;
	}

	geometry_msgs::msg::PoseStamped current_pose_msg;
	if (!current_pose(current_pose_msg)) {
		return;
	}

	const auto & target = current_goal_.pose;
	const double dx = target.position.x - current_pose_msg.pose.position.x;
	const double dy = target.position.y - current_pose_msg.pose.position.y;
	const double distance = std::hypot(dx, dy);

	if (distance <= goal_tolerance_) {
		publish_cmd_vel(0.0, 0.0);
		moving_ = false;
		has_goal_ = false;
		request_next_goal();
		return;
	}

	const double target_yaw = std::atan2(dy, dx);
	const double current_yaw = quaternion_yaw(current_pose_msg.pose.orientation);
	double yaw_error = normalize_yaw(target_yaw - current_yaw);

	if (std::abs(yaw_error) > turn_tolerance_) {
		publish_cmd_vel(0.0, angular_speed_ * std::copysign(1.0, yaw_error));
		return;
	}

	publish_cmd_vel(linear_speed_, 0.0);
}

bool WombatNode::current_pose(geometry_msgs::msg::PoseStamped & pose)
{
	try {
		const auto transform = tf_buffer_->lookupTransform(
			global_frame_, robot_base_frame_, tf2::TimePointZero);
		pose.header = transform.header;
		pose.pose.position.x = transform.transform.translation.x;
		pose.pose.position.y = transform.transform.translation.y;
		pose.pose.position.z = transform.transform.translation.z;
		pose.pose.orientation = transform.transform.rotation;
		return true;
	} catch (const tf2::TransformException & error) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Waiting for %s -> %s transform: %s",
			global_frame_.c_str(), robot_base_frame_.c_str(), error.what());
		return false;
	}
}

void WombatNode::publish_cmd_vel(double linear_speed, double angular_speed)
{
	geometry_msgs::msg::Twist msg;
	msg.linear.x = linear_speed;
	msg.angular.z = angular_speed;
	cmd_vel_pub_->publish(msg);
}

double WombatNode::normalize_yaw(double yaw)
{
	while (yaw > M_PI) {
		yaw -= 2.0 * M_PI;
	}
	while (yaw < -M_PI) {
		yaw += 2.0 * M_PI;
	}
	return yaw;
}

double WombatNode::quaternion_yaw(const geometry_msgs::msg::Quaternion & orientation)
{
	tf2::Quaternion q;
	tf2::fromMsg(orientation, q);
	tf2::Matrix3x3 m(q);
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}
