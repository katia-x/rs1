#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "rs1/srv/get_cube_centre.hpp"

/**
 * SonarNode (depth-camera version)
 *
 * Uses /parrot1/camera/depth/points (PointCloud2) + odometry to:
 *  - Estimate height above the surface (median range of points under the drone)
 *  - Estimate the cube/surface centre (mean X/Y of those points)
 *
 * Provides:
 *  - Service  /glider/sonar              (GetCubeCentre)
 *  - Topic    /glider/height_above_poo   (synthetic sensor_msgs/Range)
 */
class SonarNode : public rclcpp::Node
{
public:
  SonarNode();

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void handleGetCubeCentre(
    const std::shared_ptr<rs1::srv::GetCubeCentre::Request> request,
    std::shared_ptr<rs1::srv::GetCubeCentre::Response> response);
  
  bool use_goal_gate_{false};
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_xy_radius_{1.0};
  double closest_fraction_{0.25};
  int min_points_{20};
  // Latest odometry
  bool have_odom_{false};
  double drone_x_{0.0};
  double drone_y_{0.0};
  double drone_z_{0.0};

  // Latest processed result
  bool have_valid_measurement_{false};
  double last_range_{0.0};
  double centre_x_{0.0};
  double centre_y_{0.0};
  double centre_z_{0.0};

  // Parameters
  std::string cloud_topic_;
  std::string odom_topic_;
  double max_lateral_distance_;   // only use points within this XY distance of drone
  double min_range_;
  double max_range_;
  double range_publish_rate_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr height_pub_;
  rclcpp::Service<rs1::srv::GetCubeCentre>::SharedPtr centre_srv_;
  rclcpp::TimerBase::SharedPtr height_timer_;
};