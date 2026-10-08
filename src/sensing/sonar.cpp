#include "sonar.h"
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;

SonarNode::SonarNode()
: Node("sonar_node")
{
  this->declare_parameter("range_topic", "/parrot1/sonar");
  this->declare_parameter("odom_topic", "/parrot1/odometry");
  this->declare_parameter("min_valid_range", 0.2);
  this->declare_parameter("max_valid_range", 20.0);

  range_topic_     = this->get_parameter("range_topic").as_string();
  odom_topic_      = this->get_parameter("odom_topic").as_string();
  min_valid_range_ = this->get_parameter("min_valid_range").as_double();
  max_valid_range_ = this->get_parameter("max_valid_range").as_double();

  range_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
    range_topic_, 10, std::bind(&SonarNode::rangeCallback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 20, std::bind(&SonarNode::odomCallback, this, _1));

  // Service instead of a continuous topic
  centre_srv_ = this->create_service<rs1::srv::GetCubeCentre>(
    "/glider/sonar",
    std::bind(&SonarNode::handleGetCubeCentre, this, _1, _2));

  RCLCPP_INFO(this->get_logger(),
    "Sonar node ready. Service available at /glider/sonar");
}

void SonarNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  drone_x_ = msg->pose.pose.position.x;
  drone_y_ = msg->pose.pose.position.y;
  drone_z_ = msg->pose.pose.position.z;
  have_odom_ = true;
}

void SonarNode::rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  if (!std::isfinite(msg->range) ||
      msg->range < msg->min_range ||
      msg->range > msg->max_range ||
      msg->range < min_valid_range_ ||
      msg->range > max_valid_range_)
  {
    have_range_ = false;
    return;
  }

  last_range_ = msg->range;
  have_range_ = true;
}

void SonarNode::handleGetCubeCentre(
  const std::shared_ptr<rs1::srv::GetCubeCentre::Request> /*request*/,
  std::shared_ptr<rs1::srv::GetCubeCentre::Response> response)
{
  if (!have_odom_ || !have_range_) {
    response->success = false;
    response->message = "No valid odometry or range available";
    return;
  }

  response->success = true;
  response->message = "OK";
  response->centre.position.x = drone_x_;
  response->centre.position.y = drone_y_;
  response->centre.position.z = drone_z_ - last_range_;
  response->centre.orientation.w = 1.0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarNode>());
  rclcpp::shutdown();
  return 0;
}