#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/range.hpp>

// Change this include to match the name you generate
#include "rs1/srv/get_cube_centre.hpp"

class SonarNode : public rclcpp::Node
{
public:
  SonarNode();

private:
  void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void handleGetCubeCentre(
    const std::shared_ptr<rs1::srv::GetCubeCentre::Request> request,
    std::shared_ptr<rs1::srv::GetCubeCentre::Response> response);

  // Latest data
  bool have_odom_{false};
  bool have_range_{false};
  double drone_x_{0.0};
  double drone_y_{0.0};
  double drone_z_{0.0};
  double last_range_{0.0};

  // Parameters
  std::string range_topic_;
  std::string odom_topic_;
  double min_valid_range_;
  double max_valid_range_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<rs1::srv::GetCubeCentre>::SharedPtr centre_srv_;
};