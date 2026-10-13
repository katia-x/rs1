#ifndef RS1_MISSION_PROCESSING_H
#define RS1_MISSION_PROCESSING_H

#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
	void laser_scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
	sensor_msgs::msg::LaserScan::ConstSharedPtr latest_laser_scan_;
};

#endif  // RS1_MISSION_PROCESSING_H