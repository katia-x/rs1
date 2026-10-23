#include "mission_node.h"

MissionNode::MissionNode()
: Node("mission_node"),
  mission_processing_(*this)
{
    // Temporary direct publisher for testing commands on the Husky.
    velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        "/husky1/cmd_vel", 10);
}

void MissionNode::publish_velocity_command(
    const geometry_msgs::msg::Twist & command)
{
    // Temporary publish path until commands are connected to the mission loop.
    velocity_publisher_->publish(command);
}