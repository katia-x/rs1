#ifndef RS1_MISSION_PROCESSING_H
#define RS1_MISSION_PROCESSING_H

#include <memory>
#include <deque>
#include <vector>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/*
Mission will be our overall system orchestration layer
*/

class MissionProcessing
{
public:
	explicit MissionProcessing(rclcpp::Node & node);

	// Converts bounding box coordinates to a velocity command.
	// The bounding box is defined by the minimum and maximum x and y coordinates temporarily.
	
	geometry_msgs::msg::Twist pinpointPlant(
		float xmin,
		float ymin,
		float xmax,
		float ymax,
		const sensor_msgs::msg::LaserScan & scan);

private:
	void depth_point_cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud);
	void check_for_weeds();
	void laser_scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
	void bounding_box_callback(vision_msgs::msg::Detection2DArray::ConstSharedPtr detections);

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
	sensor_msgs::msg::LaserScan::ConstSharedPtr latest_laser_scan_;

	rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr bounding_box_subscription_;
	vision_msgs::msg::Detection2DArray::ConstSharedPtr latest_bounding_boxes_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
	std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> point_cloud_history_;
	rclcpp::TimerBase::SharedPtr weed_check_timer_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
	std::vector<geometry_msgs::msg::Point> stored_weed_coordinates_;
};

#endif  // RS1_MISSION_PROCESSING_H