#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "mission_processing.h"

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

}  // namespace

TEST(MissionProcessingTest, PinpointPlantCenteredTargetMovesForward)
{

  const auto scan = makeScanForPoint(2.0, 0.0);
  const float xmin = 330.0f;
  const float ymin = 200.0f;
  const float xmax = 390.0f;
  const float ymax = 280.0f;

  const auto cmd = MissionProcessing::pinpointPlant(xmin, ymin, xmax, ymax, scan);

  EXPECT_NEAR(cmd.linear.x, 0.3, 1e-6);
  EXPECT_NEAR(cmd.linear.y, 0.0, 1e-6);
  EXPECT_NEAR(cmd.linear.z, 0.0, 1e-6);
  EXPECT_NEAR(cmd.angular.x, 0.0, 1e-6);
  EXPECT_NEAR(cmd.angular.y, 0.0, 1e-6);
  EXPECT_NEAR(cmd.angular.z, 0.0, 1e-6);
}

TEST(MissionProcessingTest, PinpointPlantLeftTargetTurnsLeft)
{

  const auto scan = makeScanForPoint(2.0, 0.22);

  const float xmin = 300.0f;
  const float ymin = 200.0f;
  const float xmax = 360.0f;
  const float ymax = 280.0f;

  const auto cmd = MissionProcessing::pinpointPlant(xmin, ymin, xmax, ymax, scan);

  EXPECT_NEAR(cmd.linear.x, 0.0, 1e-6);
  EXPECT_GT(cmd.angular.z, 0.0);
  EXPECT_LT(std::abs(cmd.angular.z), 0.5);
}

TEST(MissionProcessingTest, PinpointPlantRightTargetTurnsRight)
{

  const auto scan = makeScanForPoint(2.0, -0.22);

  const float xmin = 360.0f;
  const float ymin = 200.0f;
  const float xmax = 420.0f;
  const float ymax = 280.0f;

  const auto cmd = MissionProcessing::pinpointPlant(xmin, ymin, xmax, ymax, scan);

  EXPECT_NEAR(cmd.linear.x, 0.0, 1e-6);
  EXPECT_LT(cmd.angular.z, 0.0);
  EXPECT_LT(std::abs(cmd.angular.z), 0.5);
}

TEST(MissionProcessingTest, PinpointPlantInvalidBoxReturnsZeroVelocity)
{

  const auto scan = makeScanForPoint(2.0, 0.0);

  const auto cmd = MissionProcessing::pinpointPlant(400.0f, 100.0f, 200.0f, 200.0f, scan);

  EXPECT_EQ(cmd.linear.x, 0.0);
  EXPECT_EQ(cmd.linear.y, 0.0);
  EXPECT_EQ(cmd.linear.z, 0.0);
  EXPECT_EQ(cmd.angular.x, 0.0);
  EXPECT_EQ(cmd.angular.y, 0.0);
  EXPECT_EQ(cmd.angular.z, 0.0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
