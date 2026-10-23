#ifndef RS1_MISSION_NODE_H
#define RS1_MISSION_NODE_H

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mission_processing.h"

class MissionNode : public rclcpp::Node
{
public:
	MissionNode();

	// TEMP command publishing interface until mission orchestration is complete.
	void publish_velocity_command(const geometry_msgs::msg::Twist & command);

private:
	MissionProcessing mission_processing_;

	// TEMP direct publisher for testing Husky velocity commands.
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
};

#endif  // RS1_MISSION_NODE_H