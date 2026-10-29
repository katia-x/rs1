#include "sonar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

SonarNode::SonarNode()
: Node("sonar_node")
{
  // ------------------------------------------------------------------
  // Parameters
  // ------------------------------------------------------------------
  this->declare_parameter("cloud_topic", "/parrot1/camera/depth/points");
  this->declare_parameter("odom_topic", "/parrot1/odometry");
  this->declare_parameter("max_lateral_distance", 0.6);   // metres
  this->declare_parameter("min_range", 0.15);
  this->declare_parameter("max_range", 20.0);
  this->declare_parameter("range_publish_rate", 20.0);
  // Optional: only use points near a known poo/cube goal
  this->declare_parameter("use_goal_gate", false);
  this->declare_parameter("goal_x", 0.0);
  this->declare_parameter("goal_y", 0.0);
  this->declare_parameter("goal_xy_radius", 1.0);

  // Prefer closest fraction of points (cube top vs ground)
  this->declare_parameter("closest_fraction", 0.25);  // keep closest 25%
  this->declare_parameter("min_points", 20);

  cloud_topic_          = this->get_parameter("cloud_topic").as_string();
  odom_topic_           = this->get_parameter("odom_topic").as_string();
  max_lateral_distance_ = this->get_parameter("max_lateral_distance").as_double();
  min_range_            = this->get_parameter("min_range").as_double();
  max_range_            = this->get_parameter("max_range").as_double();
  range_publish_rate_   = this->get_parameter("range_publish_rate").as_double();

  use_goal_gate_   = this->get_parameter("use_goal_gate").as_bool();
  goal_x_          = this->get_parameter("goal_x").as_double();
  goal_y_          = this->get_parameter("goal_y").as_double();
  goal_xy_radius_  = this->get_parameter("goal_xy_radius").as_double();
  closest_fraction_= this->get_parameter("closest_fraction").as_double();
  min_points_      = this->get_parameter("min_points").as_int();
  // ------------------------------------------------------------------
  // Subscribers
  // ------------------------------------------------------------------
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, 10,
    std::bind(&SonarNode::cloudCallback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 20,
    std::bind(&SonarNode::odomCallback, this, _1));

  // ------------------------------------------------------------------
  // Publisher – synthetic Range for the glider height logic
  // ------------------------------------------------------------------
  height_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
    "/glider/height_above_poo", 10);

  // Publish the last valid range at a fixed rate
  height_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / range_publish_rate_),
    [this]() {
      if (!have_valid_measurement_) {
        return;
      }
      sensor_msgs::msg::Range msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = "parrot1/camera_link";   // adjust if needed
      msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
      msg.field_of_view = 0.2;
      msg.min_range = min_range_;
      msg.max_range = max_range_;
      msg.range = last_range_;
      height_pub_->publish(msg);
    });

  // ------------------------------------------------------------------
  // Service
  // ------------------------------------------------------------------
  centre_srv_ = this->create_service<rs1::srv::GetCubeCentre>(
    "/glider/sonar",
    std::bind(&SonarNode::handleGetCubeCentre, this, _1, _2));

  RCLCPP_INFO(this->get_logger(),
    "Sonar node ready (depth points). Service: /glider/sonar");
}

void SonarNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  drone_x_ = msg->pose.pose.position.x;
  drone_y_ = msg->pose.pose.position.y;
  drone_z_ = msg->pose.pose.position.z;
  have_odom_ = true;
}

void SonarNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!have_odom_) {
    return;
  }

  struct Sample {
    double range;
    double wx, wy, wz;
  };
  std::vector<Sample> samples;
  samples.reserve(2000);

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = static_cast<double>(*iter_x);  // optical frame
      const double y = static_cast<double>(*iter_y);
      const double z = static_cast<double>(*iter_z);

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      // Downward camera: z ≈ range to surface
      if (z < min_range_ || z > max_range_) {
        continue;
      }

      // Near nadir only
      const double lateral = std::hypot(x, y);
      if (lateral > max_lateral_distance_) {
        continue;
      }

      // Approximate world position (nadir camera near drone centre)
      const double wx = drone_x_ + x;
      const double wy = drone_y_ + y;
      const double wz = drone_z_ - z;

      // Optional: only near known cube/poo goal
      if (use_goal_gate_) {
        const double d_goal = std::hypot(wx - goal_x_, wy - goal_y_);
        if (d_goal > goal_xy_radius_) {
          continue;
        }
      }

      samples.push_back({z, wx, wy, wz});
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "PointCloud iteration failed: %s", e.what());
    return;
  }

  if (static_cast<int>(samples.size()) < min_points_) {
    // Keep last good measurement; do not clear
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Only %zu points after filters (need %d)",
      samples.size(), min_points_);
    return;
  }

  // Sort by range (closest first). Cube top is closer than surrounding ground.
  std::sort(samples.begin(), samples.end(),
    [](const Sample & a, const Sample & b) {
      return a.range < b.range;
    });

  const size_t n_keep = std::max<size_t>(
    1, static_cast<size_t>(samples.size() * closest_fraction_));

  // Median range of closest band
  std::vector<double> close_ranges;
  close_ranges.reserve(n_keep);
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  for (size_t i = 0; i < n_keep; ++i) {
    close_ranges.push_back(samples[i].range);
    sum_x += samples[i].wx;
    sum_y += samples[i].wy;
    sum_z += samples[i].wz;
  }

  std::sort(close_ranges.begin(), close_ranges.end());
  const size_t mid = close_ranges.size() / 2;
  last_range_ = (close_ranges.size() % 2 == 0)
    ? 0.5 * (close_ranges[mid - 1] + close_ranges[mid])
    : close_ranges[mid];

  centre_x_ = sum_x / static_cast<double>(n_keep);
  centre_y_ = sum_y / static_cast<double>(n_keep);
  centre_z_ = sum_z / static_cast<double>(n_keep);

  have_valid_measurement_ = true;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Sonar: %zu pts, keep %zu closest, range=%.2f centre=(%.2f, %.2f, %.2f)",
    samples.size(), n_keep, last_range_, centre_x_, centre_y_, centre_z_);
}

void SonarNode::handleGetCubeCentre(
  const std::shared_ptr<rs1::srv::GetCubeCentre::Request> /*request*/,
  std::shared_ptr<rs1::srv::GetCubeCentre::Response> response)
{
  if (!have_odom_) {
    response->success = false;
    response->message = "No odometry yet";
    return;
  }
  if (!have_valid_measurement_) {
    response->success = false;
    response->message = "No valid depth points";
    return;
  }

  response->success = true;
  response->message = "OK";
  response->centre.position.x = centre_x_;
  response->centre.position.y = centre_y_;
  response->centre.position.z = centre_z_;
  response->centre.orientation.w = 1.0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarNode>());
  rclcpp::shutdown();
  return 0;
}