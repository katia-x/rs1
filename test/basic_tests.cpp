#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

#define private public
#include "mission_processing.h"
#undef private

namespace
{

sensor_msgs::msg::LaserScan makeScanForPoint(double range, double bearing)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = bearing;
  scan.angle_max = bearing;
  scan.angle_increment = 1.0;
  scan.range_min = 0.0;
  scan.range_max = 10.0;
  scan.ranges = std::vector<float>{static_cast<float>(range)};
  return scan;
}

sensor_msgs::msg::PointCloud2 makePointCloud()
{
  sensor_msgs::msg::PointCloud2 point_cloud;
  point_cloud.header.frame_id = "map";
  point_cloud.width = 5;
  point_cloud.height = 5;
  point_cloud.is_dense = false;
  point_cloud.fields.resize(3);
  point_cloud.fields[0].name = "x";
  point_cloud.fields[0].offset = 0;
  point_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud.fields[0].count = 1;
  point_cloud.fields[1].name = "y";
  point_cloud.fields[1].offset = 4;
  point_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud.fields[1].count = 1;
  point_cloud.fields[2].name = "z";
  point_cloud.fields[2].offset = 8;
  point_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud.fields[2].count = 1;
  point_cloud.point_step = 12;
  point_cloud.row_step = point_cloud.width * point_cloud.point_step;
  point_cloud.data.resize(point_cloud.row_step * point_cloud.height);

  sensor_msgs::PointCloud2Iterator<float> x(point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(point_cloud, "z");
  for (std::size_t index = 0; index < point_cloud.width * point_cloud.height; ++index) {
    *x = static_cast<float>(index);
    *y = static_cast<float>(index * 10);
    *z = static_cast<float>(index * 100);
    ++x;
    ++y;
    ++z;
  }

  // One sampled point is invalid; the average should use the remaining valid points.
  sensor_msgs::PointCloud2Iterator<float> invalid_x(point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> invalid_y(point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> invalid_z(point_cloud, "z");
  invalid_x += 0;
  invalid_y += 0;
  invalid_z += 0;
  *invalid_x = std::numeric_limits<float>::quiet_NaN();
  *invalid_y = std::numeric_limits<float>::quiet_NaN();
  *invalid_z = std::numeric_limits<float>::quiet_NaN();
  return point_cloud;
}

vision_msgs::msg::Detection2D makeWeedDetection(
  double center_x, double center_y, double size_x, double size_y)
{
  vision_msgs::msg::Detection2D detection;
  detection.bbox.center.position.x = center_x;
  detection.bbox.center.position.y = center_y;
  detection.bbox.size_x = size_x;
  detection.bbox.size_y = size_y;
  vision_msgs::msg::ObjectHypothesisWithPose result;
  result.hypothesis.class_id = "Weed";
  result.hypothesis.score = 0.9;
  detection.results.push_back(result);
  return detection;
}

}  // namespace

TEST(MissionProcessingTest, PinpointPlantCenteredTargetMovesForward)
{
  rclcpp::Node node("mission_processing_test");
  MissionProcessing mission_processing(node);

  const auto scan = makeScanForPoint(2.0, 0.0);
  const float xmin = 330.0f;
  const float ymin = 200.0f;
  const float xmax = 390.0f;
  const float ymax = 280.0f;

  const auto cmd = mission_processing.pinpointPlant(xmin, ymin, xmax, ymax, scan);

  EXPECT_NEAR(cmd.linear.x, 0.3, 1e-6);
  EXPECT_NEAR(cmd.linear.y, 0.0, 1e-6);
  EXPECT_NEAR(cmd.linear.z, 0.0, 1e-6);
  EXPECT_NEAR(cmd.angular.x, 0.0, 1e-6);
  EXPECT_NEAR(cmd.angular.y, 0.0, 1e-6);
  EXPECT_NEAR(cmd.angular.z, 0.0, 1e-6);
}

TEST(MissionProcessingTest, PinpointPlantLeftTargetTurnsLeft)
{
  rclcpp::Node node("mission_processing_test");
  MissionProcessing mission_processing(node);

  const auto scan = makeScanForPoint(2.0, 0.22);

  const float xmin = 300.0f;
  const float ymin = 200.0f;
  const float xmax = 360.0f;
  const float ymax = 280.0f;

  const auto cmd = mission_processing.pinpointPlant(xmin, ymin, xmax, ymax, scan);

  EXPECT_NEAR(cmd.linear.x, 0.0, 1e-6);
  EXPECT_GT(cmd.angular.z, 0.0);
  EXPECT_LT(std::abs(cmd.angular.z), 0.5);
}

TEST(MissionProcessingTest, PinpointPlantRightTargetTurnsRight)
{
  rclcpp::Node node("mission_processing_test");
  MissionProcessing mission_processing(node);

  const auto scan = makeScanForPoint(2.0, -0.22);

  const float xmin = 360.0f;
  const float ymin = 200.0f;
  const float xmax = 420.0f;
  const float ymax = 280.0f;

  const auto cmd = mission_processing.pinpointPlant(xmin, ymin, xmax, ymax, scan);

  EXPECT_NEAR(cmd.linear.x, 0.0, 1e-6);
  EXPECT_LT(cmd.angular.z, 0.0);
  EXPECT_LT(std::abs(cmd.angular.z), 0.5);
}

TEST(MissionProcessingTest, PinpointPlantInvalidBoxReturnsZeroVelocity)
{
  rclcpp::Node node("mission_processing_test");
  MissionProcessing mission_processing(node);

  const auto scan = makeScanForPoint(2.0, 0.0);

  const auto cmd = mission_processing.pinpointPlant(400.0f, 100.0f, 200.0f, 200.0f, scan);

  EXPECT_EQ(cmd.linear.x, 0.0);
  EXPECT_EQ(cmd.linear.y, 0.0);
  EXPECT_EQ(cmd.linear.z, 0.0);
  EXPECT_EQ(cmd.angular.x, 0.0);
  EXPECT_EQ(cmd.angular.y, 0.0);
  EXPECT_EQ(cmd.angular.z, 0.0);
}

TEST(MissionProcessingTest, AveragesValidDepthSamplesAcrossBoundingBoxes)
{
  rclcpp::Node node("mission_processing_depth_test");
  MissionProcessing mission_processing(node);

  auto point_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(makePointCloud());
  auto detections = std::make_shared<vision_msgs::msg::Detection2DArray>();
  detections->detections.push_back(makeWeedDetection(1.0, 1.0, 2.0, 2.0));
  detections->detections.push_back(makeWeedDetection(3.0, 3.0, 2.0, 2.0));

  mission_processing.depth_point_cloud_callback(point_cloud);
  mission_processing.bounding_box_callback(detections);
  mission_processing.check_for_weeds();

  ASSERT_EQ(mission_processing.stored_weed_coordinates_.size(), 2u);

  // The first box samples pixels (0, 0) through (2, 2); pixel (0, 0) is invalid.
  EXPECT_NEAR(mission_processing.stored_weed_coordinates_[0].x, 6.75, 1e-6);
  EXPECT_NEAR(mission_processing.stored_weed_coordinates_[0].y, 67.5, 1e-6);
  EXPECT_NEAR(mission_processing.stored_weed_coordinates_[0].z, 675.0, 1e-6);

  // The second box samples pixels (2, 2) through (4, 4).
  EXPECT_NEAR(mission_processing.stored_weed_coordinates_[1].x, 18.0, 1e-6);
  EXPECT_NEAR(mission_processing.stored_weed_coordinates_[1].y, 180.0, 1e-6);
  EXPECT_NEAR(mission_processing.stored_weed_coordinates_[1].z, 1800.0, 1e-6);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
