#include "sonar.h"

#include <algorithm>
#include <cmath>
#include <vector>

using std::placeholders::_1;

SonarNode::SonarNode()
: Node("sonar_node")
{
  this->declare_parameter("cloud_topic", "/parrot1/camera/depth/points");
  this->declare_parameter("max_lateral_distance", 5.0);
  this->declare_parameter("centre_radius", 0.4);
  this->declare_parameter("cube_half_size", 0.25);   // NEW – tight box for cube
  this->declare_parameter("min_range", 0.2);
  this->declare_parameter("max_range", 20.0);
  this->declare_parameter("high_point_margin", 0.08);
  this->declare_parameter("min_points", 30);
  this->declare_parameter("min_high_points", 10);
  this->declare_parameter("cube_search_radius", 1.5);
  this->declare_parameter("cube_max_age", 1.0);

  cloud_topic_           = this->get_parameter("cloud_topic").as_string();
  max_lateral_distance_  = this->get_parameter("max_lateral_distance").as_double();
  centre_radius_         = this->get_parameter("centre_radius").as_double();
  cube_half_size_        = this->get_parameter("cube_half_size").as_double();
  min_range_             = this->get_parameter("min_range").as_double();
  max_range_             = this->get_parameter("max_range").as_double();
  high_point_margin_     = this->get_parameter("high_point_margin").as_double();
  min_points_            = this->get_parameter("min_points").as_int();
  min_high_points_       = this->get_parameter("min_high_points").as_int();
  cube_search_radius_    = this->get_parameter("cube_search_radius").as_double();
  cube_max_age_          = this->get_parameter("cube_max_age").as_double();

  cube_service_ = this->create_service<rs1::srv::GetCubeCentre>(
    "/glider/sonar",
    std::bind(&SonarNode::getCubeCentre, this,
              std::placeholders::_1, std::placeholders::_2));

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
    "Sonar ready (floor = wide window, cube top = tight ±%.2f m box). "
    "Topics: /glider/height_above_poo, /glider/sonar_high_points, "
    "/glider/sonar_high_range",
    cube_half_size_);
}

void SonarNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Wide window  → floor height
  // Tight ±cube_half_size_ box → cube top & cube centre
  std::vector<double> floor_ranges;
  std::vector<double> centre_ranges;
  floor_ranges.reserve(2000);

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = static_cast<double>(*iter_x);
      const double y = static_cast<double>(*iter_y);
      const double z = static_cast<double>(*iter_z);

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const double depth = x;   // optical frame depth (adjust if your frame differs)
      if (depth < min_range_ || depth > max_range_) {
        continue;
      }

      const double lateral = std::hypot(y, z);
      if (lateral > max_lateral_distance_) {
        continue;
      }

      // Always collect for the floor
      floor_ranges.push_back(depth);

      // STRICT axis-aligned box for anything that influences the cube
      if (std::abs(y) < cube_half_size_ && std::abs(z) < cube_half_size_) {
        centre_ranges.push_back(depth);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "PointCloud iteration failed: %s", e.what());
    return;
  }

  if (static_cast<int>(floor_ranges.size()) < min_points_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Only %zu valid points (need %d)", floor_ranges.size(), min_points_);
    return;
  }

  // Percentile of an (unsorted) vector; sorts it in place.
  auto percentile = [](std::vector<double> & v, double p) {
    std::sort(v.begin(), v.end());
    return v[static_cast<size_t>(p * static_cast<double>(v.size() - 1))];
  };

  // Floor = far end of the wide window (90th percentile ignores stray far pixels)
  const double floor = percentile(floor_ranges, 0.90);

  // Cube top = near end of the STRICT centre box
  // 10th percentile is a bit more robust than 5th against a few noisy near points
  const bool have_centre = static_cast<int>(centre_ranges.size()) >= min_high_points_;
  const double closest = have_centre ? percentile(centre_ranges, 0.10) : floor;

  const double cube_height = floor - closest;
  const bool saw_high = have_centre && cube_height > high_point_margin_;
  const double high_range = saw_high ? closest : floor;

  // ------------------------------------------------------------------
  // Cube centre: centroid of the "high" points that lie inside the
  // same strict ±cube_half_size_ box
  // ------------------------------------------------------------------
  latest_cube_.valid = false;
  if (saw_high) {
    sensor_msgs::PointCloud2ConstIterator<float> cx(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> cy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> cz(*msg, "z");

    double sum_y = 0.0, sum_z = 0.0;
    size_t count = 0;

    for (; cx != cx.end(); ++cx, ++cy, ++cz) {
      const double d = static_cast<double>(*cx);
      const double y = static_cast<double>(*cy);
      const double z = static_cast<double>(*cz);

      if (!std::isfinite(d) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (d < min_range_ || d > floor - high_point_margin_) {
        continue;   // not closer than the floor → not part of the cube
      }

      // MUST be inside the ±0.25 m box – this is the key change
      if (std::abs(y) >= cube_half_size_ || std::abs(z) >= cube_half_size_) {
        continue;
      }

      sum_y += y;
      sum_z += z;
      ++count;
    }

    if (count >= static_cast<size_t>(min_high_points_)) {
      latest_cube_.valid   = true;
      latest_cube_.forward = sum_z / static_cast<double>(count);
      latest_cube_.left    = sum_y / static_cast<double>(count);
      latest_cube_.depth   = closest;
      latest_cube_.stamp   = this->now();
    }
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
  f.data = static_cast<float>(high_range);
  high_range_pub_->publish(f);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "floor=%.2f m | closest=%.2f m | cube_height=%.2f m | high_points=%s | "
    "n=%zu centre_n=%zu",
    floor, closest, cube_height, saw_high ? "YES" : "no",
    floor_ranges.size(), centre_ranges.size());
}

void SonarNode::getCubeCentre(
  const std::shared_ptr<rs1::srv::GetCubeCentre::Request> /*request*/,
  std::shared_ptr<rs1::srv::GetCubeCentre::Response> response)
{
  const bool fresh = latest_cube_.valid &&
    (this->now() - latest_cube_.stamp).seconds() <= cube_max_age_;

  if (!fresh) {
    response->success = false;
    response->message = "No cube currently visible";
    return;
  }

  response->success = true;
  response->message = "ok";
  response->centre.position.x = latest_cube_.forward;  // body frame, forward
  response->centre.position.y = latest_cube_.left;     // body frame, left
  response->centre.position.z = latest_cube_.depth;    // distance down to cube top
  response->centre.orientation.w = 1.0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarNode>());
  rclcpp::shutdown();
  return 0;
}