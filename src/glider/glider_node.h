#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/range.hpp>

#include <std_msgs/msg/string.hpp>

#include "glider_processing.h"

#include "rs1/srv/get_cube_centre.hpp"




class GliderNode : public rclcpp::Node
{
public:

  GliderNode();


private:

  // ROS callbacks
  void trajectoryCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg);

  void heightCallback(
    const sensor_msgs::msg::Range::SharedPtr msg);

  void controlLoop();


  // Processing layer
  std::unique_ptr<glider::GliderProcessing> processing_;


  // Subscribers
  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr traj_sub_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Subscription<
    sensor_msgs::msg::Range>::SharedPtr height_sub_;


  // Publishers
  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr status_pub_;
    
  std::string last_status_;

  // Client (service)
  //rclcpp::Client<rs1::srv::GetCubeCentre>::SharedPtr sonar_client_;


  // Control timer
  rclcpp::TimerBase::SharedPtr timer_;
};
