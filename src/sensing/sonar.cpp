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

  cloud_topic_          = this->get_parameter("cloud_topic").as_string();
  odom_topic_           = this->get_parameter("odom_topic").as_string();
  max_lateral_distance_ = this->get_parameter("max_lateral_distance").as_double();
  min_range_            = this->get_parameter("min_range").as_double();
  max_range_            = this->get_parameter("max_range").as_double();
  range_publish_rate_   = this->get_parameter("range_publish_rate").as_double();

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

  // Collect ranges and world-frame positions of points under the drone
  std::vector<double> ranges;
  std::vector<double> xs, ys, zs;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;   // camera / optical frame
      const float y = *iter_y;
      const float z = *iter_z;

      // Skip invalid points
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      // Depth cameras usually have +Z forward. For a downward camera
      // the "range" is roughly the Z value (or the 3D distance).
      // We treat z as the distance along the optical axis.
      const double range = static_cast<double>(z);
      if (range < min_range_ || range > max_range_) {
        continue;
      }

      // Only keep points that are roughly under the drone
      // (small lateral offset in the camera frame)
      const double lateral = std::hypot(static_cast<double>(x), static_cast<double>(y));
      if (lateral > max_lateral_distance_) {
        continue;
      }

      ranges.push_back(range);

      // Approximate world position of the point:
      // assume the camera is looking straight down and is close to the drone centre.
      // (Good enough for a first version; can be improved with TF later.)
      xs.push_back(drone_x_ + static_cast<double>(x));
      ys.push_back(drone_y_ + static_cast<double>(y));
      zs.push_back(drone_z_ - range);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "PointCloud iteration failed: %s", e.what());
    return;
  }

  if (ranges.empty()) {
    have_valid_measurement_ = false;
    return;
  }

  // Median range (more robust than mean)
  std::sort(ranges.begin(), ranges.end());
  const size_t mid = ranges.size() / 2;
  last_range_ = (ranges.size() % 2 == 0)
    ? 0.5 * (ranges[mid - 1] + ranges[mid])
    : ranges[mid];

  // Mean centre
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  for (size_t i = 0; i < xs.size(); ++i) {
    sum_x += xs[i];
    sum_y += ys[i];
    sum_z += zs[i];
  }
  centre_x_ = sum_x / static_cast<double>(xs.size());
  centre_y_ = sum_y / static_cast<double>(ys.size());
  centre_z_ = sum_z / static_cast<double>(zs.size());

  have_valid_measurement_ = true;
}

void SonarNode::handleGetCubeCentre(
  const std::shared_ptr<rs1::srv::GetCubeCentre::Request> /*request*/,
  std::shared_ptr<rs1::srv::GetCubeCentre::Response> response)
{
  if (!have_odom_ || !have_valid_measurement_) {
    response->success = false;
    response->message = "No valid odometry or depth measurement available";
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