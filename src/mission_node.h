#ifndef RS1_MISSION_NODE_H_
#define RS1_MISSION_NODE_H_

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mission_processing.h"

#include "rs1/srv/wombat_next_goal.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class MissionNode : public rclcpp::Node
{
public:
	MissionNode();

private:
	void region_callback(const geometry_msgs::msg::PoseArray::SharedPtr message);
	void update_weed_target();
	void publish_weed_estimates(const rclcpp::Time & time);
	void clear_weed_target();
	bool current_pose(geometry_msgs::msg::PoseStamped & pose);
	void build_search_lines();
	void serve_next_goal(
		const std::shared_ptr<rs1::srv::WombatNextGoal::Request> request,
		std::shared_ptr<rs1::srv::WombatNextGoal::Response> response);
	static double distance(const geometry_msgs::msg::Point & first, const geometry_msgs::msg::Point & second);

	std::string global_frame_;
	std::string wombat_base_frame_;
	double search_lane_spacing_{1.25};
	double search_goal_distance_{2.5};
	bool search_line_assignment_alternation{false};
	std::size_t search_goal_index_{0};
	std::vector<geometry_msgs::msg::Pose> region_vertices_;
	std::vector<geometry_msgs::msg::PoseStamped> search_lines_;
	double location_tolerance_{0.25};
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr region_subscription_;
	rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_subscription_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr estimate_markers_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr permanent_weed_publisher_;
	rclcpp::TimerBase::SharedPtr weed_timer_;
	rclcpp::Service<rs1::srv::WombatNextGoal>::SharedPtr next_goal_service_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	std::unique_ptr<MissionProcessing> mission_processing_;
	std::optional<MissionProcessing::WeedEstimate> selected_weed_;
	std::optional<std::uint64_t> selected_weed_id_;
	std::size_t consecutive_goal_failures_{0};
	geometry_msgs::msg::Point last_goal_position_;
	bool last_goal_valid_{false};
	bool last_goal_was_weed_{false};
	bool target_visible_{false};
};

#endif
