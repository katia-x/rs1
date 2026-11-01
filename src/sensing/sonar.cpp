#include "sonar.h"

#include <algorithm>
#include <cmath>
#include <vector>

using std::placeholders::_1;

SonarNode::SonarNode()
: Node("sonar_node")
{
  this->declare_parameter("cloud_topic", "/parrot1/camera/depth/points");
  this->declare_parameter("max_lateral_distance", 1.0);
  this->declare_parameter("min_range", 0.2);
  this->declare_parameter("max_range", 12.0);
  this->declare_parameter("high_point_margin", 0.08);
  this->declare_parameter("bin_size", 0.03);
  this->declare_parameter("min_points", 30);
  this->declare_parameter("min_high_points", 10);

  cloud_topic_ = this->get_parameter("cloud_topic").as_string();
  max_lateral_distance_ = this->get_parameter("max_lateral_distance").as_double();
  min_range_ = this->get_parameter("min_range").as_double();
  max_range_ = this->get_parameter("max_range").as_double();
  high_point_margin_ = this->get_parameter("high_point_margin").as_double();
  bin_size_ = this->get_parameter("bin_size").as_double();
  min_points_ = this->get_parameter("min_points").as_int();
  min_high_points_ = this->get_parameter("min_high_points").as_int();

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, 10,
    std::bind(&SonarNode::cloudCallback, this, _1));

  floor_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
    "/glider/height_above_poo", 10);

  high_bool_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/glider/sonar_high_points", 10);

  high_range_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "/glider/sonar_high_range", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "Bare-bones sonar ready (majority/histogram floor). "
    "Topics: /glider/height_above_poo, /glider/sonar_high_points, "
    "/glider/sonar_high_range");
}

void SonarNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::vector<double> ranges;
  ranges.reserve(2000);

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = static_cast<double>(*iter_x);
      const double y = static_cast<double>(*iter_y);
      const double z = static_cast<double>(*iter_z);  // depth (down)

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < min_range_ || z > max_range_) {
        continue;
      }
      if (std::hypot(x, y) > max_lateral_distance_) {
        continue;
      }
      ranges.push_back(z);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "PointCloud iteration failed: %s", e.what());
    return;
  }

  if (static_cast<int>(ranges.size()) < min_points_) {
    return;
  }

  // ------------------------------------------------------------------
  // Floor = majority depth (histogram mode)
  // ------------------------------------------------------------------
  double z_min = ranges.front();
  double z_max = ranges.front();
  for (double r : ranges) {
    z_min = std::min(z_min, r);
    z_max = std::max(z_max, r);
  }

  const double bin = std::max(1e-3, bin_size_);
  const int n_bins = std::max(1, static_cast<int>((z_max - z_min) / bin) + 1);
  std::vector<int> hist(n_bins, 0);

  for (double r : ranges) {
    int b = static_cast<int>((r - z_min) / bin);
    if (b < 0) {
      b = 0;
    }
    if (b >= n_bins) {
      b = n_bins - 1;
    }
    hist[b]++;
  }

  int best_bin = 0;
  for (int b = 1; b < n_bins; ++b) {
    if (hist[b] > hist[best_bin]) {
      best_bin = b;
    }
  }

  const double floor = z_min + (static_cast<double>(best_bin) + 0.5) * bin;

  // ------------------------------------------------------------------
  // High points = clearly closer than majority floor (cube)
  // ------------------------------------------------------------------
  std::vector<double> high;
  for (double r : ranges) {
    if (r < floor - high_point_margin_) {
      high.push_back(r);
    }
  }

  const bool saw_high = static_cast<int>(high.size()) >= min_high_points_;
  double high_range = floor;
  if (saw_high) {
    std::sort(high.begin(), high.end());
    const size_t hm = high.size() / 2;
    high_range = (high.size() % 2 == 0)
      ? 0.5 * (high[hm - 1] + high[hm])
      : high[hm];
  }

  // ------------------------------------------------------------------
  // Publish
  // ------------------------------------------------------------------
  sensor_msgs::msg::Range range_msg;
  range_msg.header.stamp = this->now();
  range_msg.header.frame_id = "parrot1_camera_optical_frame";
  range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  range_msg.field_of_view = 0.2f;
  range_msg.min_range = static_cast<float>(min_range_);
  range_msg.max_range = static_cast<float>(max_range_);
  range_msg.range = static_cast<float>(floor);
  floor_pub_->publish(range_msg);

  std_msgs::msg::Bool b;
  b.data = saw_high;
  high_bool_pub_->publish(b);

  std_msgs::msg::Float32 f;
  f.data = static_cast<float>(saw_high ? high_range : floor);
  high_range_pub_->publish(f);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "floor=%.2f m (majority) | high_points=%s | high_range=%.2f m | n=%zu high_n=%zu",
    floor,
    saw_high ? "YES" : "no",
    high_range,
    ranges.size(),
    high.size());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarNode>());
  rclcpp::shutdown();
  return 0;
}