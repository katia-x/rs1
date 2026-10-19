#ifndef RS1_MISSION_NODE_H_
#define RS1_MISSION_NODE_H_

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rs1/msg/plant_detection_array.hpp"
#include "rs1/srv/wombat_next_goal.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

class MissionNode : public rclcpp::Node
{
public:
	MissionNode();

private:
	void region_callback(const geometry_msgs::msg::PoseArray::SharedPtr message);
	void weed_callback(const rs1::msg::PlantDetectionArray::SharedPtr message);
	bool current_pose(geometry_msgs::msg::PoseStamped & pose);
	void build_search_lines();
	void serve_next_goal(
		const std::shared_ptr<rs1::srv::WombatNextGoal::Request> request,
		std::shared_ptr<rs1::srv::WombatNextGoal::Response> response);
	static double distance(const geometry_msgs::msg::Point & first, const geometry_msgs::msg::Point & second);

	std::string global_frame_;
	std::string wombat_base_frame_;
	double camera_width_of_view_{2.5};
	double search_goal_distance_{2.5};
	bool search_line_assignment_alternation{false};
	std::size_t search_goal_index_{0};
	std::vector<geometry_msgs::msg::Pose> region_vertices_;
	std::vector<rs1::msg::PlantDetection> weeds_;
	std::vector<geometry_msgs::msg::PoseStamped> search_lines_;
	double location_tolerance_{0.25};
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr region_subscription_;
	rclcpp::Subscription<rs1::msg::PlantDetectionArray>::SharedPtr weed_subscription_;
	rclcpp::Service<rs1::srv::WombatNextGoal>::SharedPtr next_goal_service_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif