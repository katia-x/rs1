#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <random>

#include "mission_processing.h"

namespace
{
using Point = geometry_msgs::msg::Point;
using Config = MissionProcessing::Config;
using Result = MissionProcessing::UpdateResult;

Point point(double x, double y = 0.0, double z = 0.4)
{
    Point p; p.x = x; p.y = y; p.z = z; return p;
}

rclcpp::Time time_at(double seconds)
{
    return rclcpp::Time(static_cast<std::int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}

sensor_msgs::msg::PointCloud2::SharedPtr cloud_at(double seconds,
    const std::function<Point(int, int)> & pixel = [](int, int) { return point(3.0); }, int padding = 0)
{
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = "husky1_camera_link";
    cloud->header.stamp = time_at(seconds);
    cloud->width = cloud->height = 21;
    cloud->point_step = 12;
    cloud->row_step = cloud->width * cloud->point_step + padding;
    cloud->data.resize(cloud->row_step * cloud->height);
    for (std::size_t axis = 0; axis < 3; ++axis) {
        sensor_msgs::msg::PointField field;
        field.name = axis == 0 ? "x" : axis == 1 ? "y" : "z";
        field.offset = 4 * axis; field.count = 1; field.datatype = field.FLOAT32;
        cloud->fields.push_back(field);
    }
    for (int v = 0; v < 21; ++v) {
        for (int u = 0; u < 21; ++u) {
            const auto p = pixel(u, v);
            const float xyz[] = {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
            std::memcpy(cloud->data.data() + v * cloud->row_step + u * cloud->point_step, xyz, sizeof(xyz));
        }
    }
    return cloud;
}

vision_msgs::msg::Detection2DArray::SharedPtr detections_at(double seconds)
{
    auto detections = std::make_shared<vision_msgs::msg::Detection2DArray>();
    detections->header.frame_id = "husky1_camera_link";
    detections->header.stamp = time_at(seconds);
    vision_msgs::msg::Detection2D detection;
    detection.bbox.center.position.x = detection.bbox.center.position.y = 10;
    detection.bbox.size_x = detection.bbox.size_y = 20;
    vision_msgs::msg::ObjectHypothesisWithPose result;
    result.hypothesis.class_id = "Weed"; result.hypothesis.score = 0.9;
    detection.results.push_back(result);
    detections->detections.push_back(detection);
    return detections;
}

std::vector<geometry_msgs::msg::Pose> region(double max_x = 10, double max_y = 3)
{
    std::vector<geometry_msgs::msg::Pose> polygon(4);
    polygon[0].position = point(0, -3, 0); polygon[1].position = point(max_x, -3, 0);
    polygon[2].position = point(max_x, max_y, 0); polygon[3].position = point(0, max_y, 0);
    return polygon;
}

class WeedEstimationTest : public ::testing::Test
{
protected:
    std::shared_ptr<tf2_ros::Buffer> buffer = std::make_shared<tf2_ros::Buffer>(
        std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));

    void SetUp() override { set_transform(); }

    void set_transform(double tx = 0, double ty = 0, double yaw = 0, double stamp = 0, bool is_static = true)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = "map"; tf.child_frame_id = "husky1_camera_link";
        tf.header.stamp = time_at(stamp);
        tf.transform.translation.x = tx; tf.transform.translation.y = ty;
        tf.transform.rotation.z = std::sin(yaw / 2); tf.transform.rotation.w = std::cos(yaw / 2);
        ASSERT_TRUE(buffer->setTransform(tf, "test", is_static));
    }

    Result observe(MissionProcessing & processing, double seconds, Point p = point(3))
    {
        processing.receive_cloud(cloud_at(seconds, [p](int, int) { return p; }));
        processing.receive_detections(detections_at(seconds));
        return processing.update(time_at(seconds + 0.01));
    }
};

TEST_F(WeedEstimationTest, RejectsInvalidConfiguration)
{
    Config config; config.min_cluster_fraction = 0.5;
    EXPECT_THROW(MissionProcessing(config, buffer), std::invalid_argument);
    config = Config{}; config.track_timeout = -1;
    EXPECT_THROW(MissionProcessing(config, buffer), std::invalid_argument);
    config = Config{}; config.smoothing_factor = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(MissionProcessing(config, buffer), std::invalid_argument);
    EXPECT_THROW(MissionProcessing(Config{}, nullptr), std::invalid_argument);
}

TEST_F(WeedEstimationTest, BoundsTimeMatchingAndAcceptsLateMatchingCloud)
{
    MissionProcessing processing(Config{}, buffer);
    processing.receive_cloud(cloud_at(9.8));
    processing.receive_detections(detections_at(10));
    EXPECT_EQ(processing.update(time_at(10)), Result::WaitingForCloud);
    EXPECT_TRUE(processing.estimates().empty());
    const auto timing = processing.cloud_timing();
    ASSERT_TRUE(timing.detection_stamp);
    ASSERT_TRUE(timing.nearest_cloud_stamp);
    EXPECT_EQ(timing.buffered_clouds, 1u);
    EXPECT_EQ(*timing.detection_stamp, time_at(10));
    EXPECT_EQ(*timing.nearest_cloud_stamp, time_at(9.8));
    processing.receive_cloud(cloud_at(10.02));
    EXPECT_EQ(processing.update(time_at(10.03)), Result::Processed);
    ASSERT_EQ(processing.estimates().size(), 1u);
    EXPECT_EQ(processing.estimates()[0].last_seen, time_at(10.02));
    EXPECT_FALSE(processing.cloud_timing().detection_stamp);
    EXPECT_FALSE(processing.cloud_timing().nearest_cloud_stamp);
}

TEST_F(WeedEstimationTest, DistinguishesMissingCloudFromTimestampMismatch)
{
    MissionProcessing processing(Config{}, buffer);
    processing.receive_detections(detections_at(10));
    EXPECT_EQ(processing.update(time_at(10)), Result::WaitingForCloud);
    const auto timing = processing.cloud_timing();
    EXPECT_EQ(timing.buffered_clouds, 0u);
    EXPECT_TRUE(timing.detection_stamp);
    EXPECT_FALSE(timing.nearest_cloud_stamp);
}

TEST_F(WeedEstimationTest, NewDetectionsDoNotReplaceBatchWaitingForDelayedDepth)
{
    MissionProcessing processing(Config{}, buffer);
    processing.receive_cloud(cloud_at(82.918));
    processing.receive_detections(detections_at(83.584));
    ASSERT_EQ(processing.update(time_at(84.104)), Result::WaitingForCloud);

    // A newer inference result arrives before the older cloud is delivered.
    processing.receive_detections(detections_at(83.917));
    processing.receive_cloud(cloud_at(83.584));
    ASSERT_EQ(processing.update(time_at(84.2)), Result::Processed);
    ASSERT_EQ(processing.estimates().size(), 1u);
    EXPECT_EQ(processing.estimates()[0].last_seen, time_at(83.584));

    processing.receive_cloud(cloud_at(83.917));
    ASSERT_EQ(processing.update(time_at(84.3)), Result::Processed);
    EXPECT_EQ(processing.estimates()[0].last_seen, time_at(83.917));
    EXPECT_EQ(processing.estimates()[0].observation_count, 2u);
}

TEST_F(WeedEstimationTest, PendingQueueIsOrderedDeduplicatedAndBounded)
{
    MissionProcessing processing(Config{}, buffer);
    processing.receive_detections(detections_at(10.5));
    processing.receive_detections(detections_at(10));
    processing.receive_detections(detections_at(10.5));
    EXPECT_EQ(processing.cloud_timing().pending_batches, 2u);
    ASSERT_TRUE(processing.cloud_timing().detection_stamp);
    EXPECT_EQ(*processing.cloud_timing().detection_stamp, time_at(10));

    for (int i = 11; i <= 45; ++i) {
        processing.receive_detections(detections_at(i));
    }
    EXPECT_EQ(processing.cloud_timing().pending_batches, 30u);
    EXPECT_EQ(*processing.cloud_timing().detection_stamp, time_at(16));
    processing.receive_detections(detections_at(12)); // Already evicted.
    EXPECT_EQ(processing.cloud_timing().pending_batches, 30u);
}

TEST_F(WeedEstimationTest, ExpiredBatchDoesNotPreventNextBatchFromMatching)
{
    MissionProcessing processing(Config{}, buffer);
    processing.receive_detections(detections_at(10));
    processing.receive_detections(detections_at(11));
    processing.receive_cloud(cloud_at(11));
    EXPECT_EQ(processing.update(time_at(12.1)), Result::Rejected);
    EXPECT_EQ(processing.cloud_timing().pending_batches, 1u);
    ASSERT_EQ(processing.update(time_at(12.2)), Result::Processed);
    ASSERT_EQ(processing.estimates().size(), 1u);
    EXPECT_EQ(processing.estimates()[0].last_seen, time_at(11));
}

TEST_F(WeedEstimationTest, RejectsOldObservations)
{
    MissionProcessing processing(Config{}, buffer);
    processing.receive_cloud(cloud_at(10)); processing.receive_detections(detections_at(10));
    EXPECT_EQ(processing.update(time_at(12.1)), Result::Rejected);
    EXPECT_TRUE(processing.estimates().empty());
}

TEST_F(WeedEstimationTest, WaitsForFutureObservationTime)
{
    MissionProcessing processing(Config{}, buffer);
    processing.receive_cloud(cloud_at(10)); processing.receive_detections(detections_at(10));
    EXPECT_EQ(processing.update(time_at(9.9)), Result::Idle);
    EXPECT_TRUE(processing.estimates().empty());
    EXPECT_EQ(processing.update(time_at(10)), Result::Processed);
}

TEST_F(WeedEstimationTest, ProcessesBatchOnceAndIgnoresOutOfOrderDetections)
{
    MissionProcessing processing(Config{}, buffer);
    ASSERT_EQ(observe(processing, 10), Result::Processed);
    processing.receive_detections(detections_at(10));
    processing.receive_detections(detections_at(9.9));
    EXPECT_EQ(processing.update(time_at(10.2)), Result::Idle);
    ASSERT_EQ(processing.estimates().size(), 1u);
    EXPECT_EQ(processing.estimates()[0].observation_count, 1u);
}

TEST_F(WeedEstimationTest, RetriesAfterMissingHistoricTransform)
{
    buffer = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    MissionProcessing processing(Config{}, buffer);
    processing.receive_cloud(cloud_at(10)); processing.receive_detections(detections_at(10));
    EXPECT_EQ(processing.update(time_at(10)), Result::WaitingForTransform);
    set_transform(1, 2, 0);
    EXPECT_EQ(processing.update(time_at(10.1)), Result::Processed);
    ASSERT_EQ(processing.estimates().size(), 1u);
    EXPECT_NEAR(processing.estimates()[0].position.x, 4, 1e-6);
    EXPECT_NEAR(processing.estimates()[0].position.y, 2, 1e-6);
}

TEST_F(WeedEstimationTest, UsesAcquisitionTimeRotationAndTranslation)
{
    buffer = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
    set_transform(2, 1, M_PI / 2, 10, false);
    set_transform(100, 100, 0, 11, false);
    MissionProcessing processing(Config{}, buffer);
    ASSERT_EQ(observe(processing, 10, point(3, 0.2)), Result::Processed);
    ASSERT_EQ(processing.estimates().size(), 1u);
    const auto & p = processing.estimates()[0].position;
    EXPECT_NEAR(p.x, 1.8, 1e-6); EXPECT_NEAR(p.y, 4, 1e-6); EXPECT_NEAR(p.z, 0.4, 1e-6);
}

TEST_F(WeedEstimationTest, RejectsFrameMismatchAndMalformedCloud)
{
    MissionProcessing processing(Config{}, buffer);
    auto detections = detections_at(10); detections->header.frame_id = "other_camera";
    processing.receive_cloud(cloud_at(10)); processing.receive_detections(detections);
    EXPECT_EQ(processing.update(time_at(10)), Result::Rejected);
    auto cloud = cloud_at(11); cloud->data.resize(3);
    processing.receive_cloud(cloud); processing.receive_detections(detections_at(11));
    EXPECT_EQ(processing.update(time_at(11)), Result::Rejected);
    EXPECT_TRUE(processing.estimates().empty());
}

TEST_F(WeedEstimationTest, HandlesRowPaddingAndBigEndianCoordinates)
{
    MissionProcessing processing(Config{}, buffer);
    auto cloud = cloud_at(10, [](int, int) { return point(3, 0.2); }, 8);
    for (unsigned v = 0; v < cloud->height; ++v) {
        for (unsigned u = 0; u < cloud->width; ++u) {
            for (unsigned field = 0; field < 3; ++field) {
                auto start = cloud->data.begin() + v * cloud->row_step + u * cloud->point_step + field * 4;
                std::reverse(start, start + 4);
            }
        }
    }
    cloud->is_bigendian = true;
    processing.receive_cloud(cloud); processing.receive_detections(detections_at(10));
    ASSERT_EQ(processing.update(time_at(10)), Result::Processed);
    ASSERT_EQ(processing.estimates().size(), 1u);
    EXPECT_NEAR(processing.estimates()[0].position.x, 3, 1e-6);
    EXPECT_NEAR(processing.estimates()[0].position.y, 0.2, 1e-6);
}

TEST_F(WeedEstimationTest, RejectsWrongClassAndLowConfidence)
{
    MissionProcessing processing(Config{}, buffer);
    auto detections = detections_at(10);
    detections->detections[0].results[0].hypothesis.score = 0.75;
    processing.receive_cloud(cloud_at(10)); processing.receive_detections(detections);
    processing.update(time_at(10));
    detections = detections_at(11);
    detections->detections[0].results[0].hypothesis.class_id = "Plant";
    processing.receive_cloud(cloud_at(11)); processing.receive_detections(detections);
    processing.update(time_at(11));
    EXPECT_TRUE(processing.estimates().empty());
}

TEST_F(WeedEstimationTest, ExcludesBoxEdgesAndMinorityBackground)
{
    MissionProcessing processing(Config{}, buffer);
    auto cloud = cloud_at(10, [](int u, int v) {
        return u < 5 || u > 15 || v < 5 || v > 15 || (u + v) % 5 == 0 ? point(8) : point(3, 0.2);
    });
    processing.receive_cloud(cloud); processing.receive_detections(detections_at(10));
    ASSERT_EQ(processing.update(time_at(10)), Result::Processed);
    ASSERT_EQ(processing.estimates().size(), 1u);
    EXPECT_NEAR(processing.estimates()[0].position.x, 3, 1e-6);
    EXPECT_NEAR(processing.estimates()[0].position.y, 0.2, 1e-6);
}

TEST_F(WeedEstimationTest, RejectsAmbiguousDepthAndInvalidSamples)
{
    MissionProcessing processing(Config{}, buffer);
    processing.receive_cloud(cloud_at(10, [](int u, int) { return point(u < 10 ? 3 : 8); }));
    processing.receive_detections(detections_at(10)); processing.update(time_at(10));
    EXPECT_TRUE(processing.estimates().empty());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    observe(processing, 11, point(nan));
    EXPECT_TRUE(processing.estimates().empty());
    observe(processing, 12, point(-1));
    EXPECT_TRUE(processing.estimates().empty());
}

TEST_F(WeedEstimationTest, RejectsInvalidAndTinyBoundingBoxes)
{
    MissionProcessing processing(Config{}, buffer);
    for (int i = 0; i < 4; ++i) {
        auto detections = detections_at(10 + i);
        auto & box = detections->detections[0].bbox;
        if (i == 0) { box.size_x = -1; }
        if (i == 1) { box.center.position.x = 1000; }
        if (i == 2) { box.center.position.y = std::numeric_limits<double>::quiet_NaN(); }
        if (i == 3) { box.size_x = box.size_y = 0.1; }
        processing.receive_cloud(cloud_at(10 + i)); processing.receive_detections(detections);
        processing.update(time_at(10 + i));
    }
    EXPECT_TRUE(processing.estimates().empty());
}

TEST_F(WeedEstimationTest, UpdatesExistingEstimateWithSmoothing)
{
    MissionProcessing processing(Config{}, buffer);
    observe(processing, 10, point(3));
    const auto id = processing.estimates().at(0).id;
    observe(processing, 11, point(3.2));
    ASSERT_EQ(processing.estimates().size(), 1u);
    const auto & estimate = processing.estimates()[0];
    EXPECT_EQ(estimate.id, id); EXPECT_EQ(estimate.observation_count, 2u);
    EXPECT_NEAR(estimate.position.x, 3.1, 1e-6); EXPECT_EQ(estimate.last_seen, time_at(11));
}

TEST_F(WeedEstimationTest, ExpiryUsesLastObservationNotRepeatedProcessing)
{
    MissionProcessing processing(Config{}, buffer);
    observe(processing, 10);
    for (int i = 1; i <= 10; ++i) { processing.update(time_at(10 + i * 0.2)); }
    ASSERT_EQ(processing.estimates().size(), 1u);
    EXPECT_EQ(processing.estimates()[0].last_seen, time_at(10));
    processing.update(time_at(13.01));
    EXPECT_TRUE(processing.estimates().empty());
}

TEST_F(WeedEstimationTest, ReusedDepthFrameDoesNotRefreshTrack)
{
    MissionProcessing processing(Config{}, buffer);
    observe(processing, 10);
    processing.receive_detections(detections_at(10.05));
    ASSERT_EQ(processing.update(time_at(10.1)), Result::Processed);
    ASSERT_EQ(processing.estimates().size(), 1u);
    EXPECT_EQ(processing.estimates()[0].observation_count, 1u);
    EXPECT_EQ(processing.estimates()[0].last_seen, time_at(10));
    processing.update(time_at(13.1));
    processing.receive_detections(detections_at(10));
    EXPECT_EQ(processing.update(time_at(13.2)), Result::Idle);
    EXPECT_TRUE(processing.estimates().empty());
}

TEST_F(WeedEstimationTest, AssociatesNearbyWeedsOneToOnePerBatch)
{
    MissionProcessing processing(Config{}, buffer);
    for (int frame = 0; frame < 2; ++frame) {
        auto detections = detections_at(10 + frame);
        auto & first = detections->detections[0];
        first.bbox.center.position.x = 4;
        first.bbox.size_x = first.bbox.size_y = 6;
        auto second = first;
        second.bbox.center.position.x = 16;
        detections->detections.push_back(second);
        processing.receive_cloud(cloud_at(10 + frame, [frame](int u, int) {
            return point((u < 10 ? 3.0 : 3.3) + frame * 0.1);
        }));
        processing.receive_detections(detections);
        ASSERT_EQ(processing.update(time_at(10 + frame)), Result::Processed);
    }
    ASSERT_EQ(processing.estimates().size(), 2u);
    EXPECT_NEAR(processing.estimates()[0].position.x, 3.05, 1e-6);
    EXPECT_NEAR(processing.estimates()[1].position.x, 3.35, 1e-6);
    EXPECT_EQ(processing.estimates()[0].observation_count, 2u);
    EXPECT_EQ(processing.estimates()[1].observation_count, 2u);
}

TEST_F(WeedEstimationTest, ClockRewindClearsOldTracksAndAllowsNewEpoch)
{
    MissionProcessing processing(Config{}, buffer);
    observe(processing, 10);
    processing.update(time_at(0));
    EXPECT_TRUE(processing.estimates().empty());
    observe(processing, 1);
    ASSERT_EQ(processing.estimates().size(), 1u);
    EXPECT_EQ(processing.estimates()[0].last_seen, time_at(1));
}

TEST_F(WeedEstimationTest, SelectsNearestAheadWithinLaneAndRegion)
{
    MissionProcessing processing(Config{}, buffer);
    observe(processing, 10, point(1)); // Behind robot.
    observe(processing, 10.1, point(3, 2)); // Off lane.
    observe(processing, 10.2, point(11)); // Beyond lane.
    observe(processing, 10.3, point(5));
    observe(processing, 10.4, point(3));
    const MissionProcessing::SearchLane lane{point(0), point(10), 0.625};
    const auto target = processing.select_target(point(2), lane, region());
    ASSERT_TRUE(target); EXPECT_NEAR(target->position.x, 3, 1e-6);
    EXPECT_FALSE(processing.select_target(point(2), lane, region(2.5)));
}

TEST_F(WeedEstimationTest, SupportsReverseLaneAndStableTargetIdentity)
{
    MissionProcessing processing(Config{}, buffer);
    observe(processing, 10, point(7));
    observe(processing, 10.1, point(5));
    const auto preferred = processing.estimates()[1].id;
    const MissionProcessing::SearchLane lane{point(10), point(0), 0.625};
    auto target = processing.select_target(point(8), lane, region());
    ASSERT_TRUE(target); EXPECT_NEAR(target->position.x, 7, 1e-6);
    target = processing.select_target(point(8), lane, region(), preferred);
    ASSERT_TRUE(target); EXPECT_EQ(target->id, preferred);
    EXPECT_FALSE(processing.select_target(point(4), lane, region()));
}

TEST_F(WeedEstimationTest, RespectsConcaveRegionInsteadOfItsBoundingBox)
{
    MissionProcessing processing(Config{}, buffer);
    observe(processing, 10, point(3, 3));
    std::vector<geometry_msgs::msg::Pose> polygon(6);
    const Point vertices[] = {point(0,0), point(5,0), point(5,1), point(1,1), point(1,5), point(0,5)};
    for (int i = 0; i < 6; ++i) { polygon[i].position = vertices[i]; }
    EXPECT_FALSE(processing.select_target(point(0,3), {point(0,3), point(5,3), 0.625}, polygon));
}

TEST_F(WeedEstimationTest, AssignsCapturedWeedToConfiguredLaneWithoutGap)
{
    MissionProcessing processing(Config{}, buffer);
    // Map position measured during the live investigation: previously rejected
    // by every lane because 1.25 m spacing exceeded the 1.0 m corridor width.
    observe(processing, 10, point(3.091520, -3.276300, 0.091390));
    auto polygon = region(5, 5);
    polygon[0].position = point(-5, -5); polygon[1].position = point(5, -5);
    polygon[3].position = point(-5, 5);
    const auto robot = point(4.007986, 1.007450);
    const MissionProcessing::SearchLane lane{point(2.5, 5), point(2.5, -5), 1.25 / 2};
    EXPECT_FALSE(processing.select_target(robot, {lane.start, lane.end, 0.5}, polygon));
    ASSERT_TRUE(processing.select_target(robot, lane, polygon));
    // Estimating a weed does not make it eligible on a different current lane.
    EXPECT_FALSE(processing.select_target(robot, {point(-3.75, 5), point(-3.75, -5), lane.half_width}, polygon));
    EXPECT_FALSE(processing.select_target(robot, {point(3.75, 5), point(3.75, -5), lane.half_width}, polygon));
}

TEST_F(WeedEstimationTest, AdjacentLaneCorridorsCoverTheirSharedStrip)
{
    constexpr double spacing = 1.25;
    const MissionProcessing::SearchLane left{point(2.5, -3), point(2.5, 3), spacing / 2};
    const MissionProcessing::SearchLane right{point(3.75, -3), point(3.75, 3), spacing / 2};
    for (int step = 0; step <= 128; ++step) {
        MissionProcessing processing(Config{}, buffer);
        const double x = left.start.x + spacing * step / 128;
        observe(processing, 10, point(x, 1));
        const auto in_left = processing.select_target(point(2.5, 0), left, region());
        const auto in_right = processing.select_target(point(3.75, 0), right, region());
        EXPECT_TRUE(in_left || in_right) << "Uncovered x=" << x;
        if (step == 64) { EXPECT_TRUE(in_left); EXPECT_TRUE(in_right); }
    }
}

TEST_F(WeedEstimationTest, RejectsInvalidLaneWidth)
{
    MissionProcessing processing(Config{}, buffer);
    observe(processing, 10);
    for (const double width : {0.0, -1.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_FALSE(processing.select_target(point(0), {point(0), point(10), width}, region()));
    }
}

TEST_F(WeedEstimationTest, SyntheticNoisyForegroundAccuracy)
{
    std::mt19937 generator(42);
    std::normal_distribution<double> noise(0.0, 0.02);
    const auto truth = point(3, 0.2, 0.4);
    double sum_squared_error = 0, max_error = 0;
    for (int trial = 0; trial < 100; ++trial) {
        MissionProcessing processing(Config{}, buffer);
        processing.receive_cloud(cloud_at(10, [&](int u, int v) {
            if (u < 5 || u > 15 || v < 5 || v > 15 || (u + v) % 5 == 0) { return point(8); }
            return point(truth.x + noise(generator), truth.y + noise(generator), truth.z + noise(generator));
        }));
        processing.receive_detections(detections_at(10)); processing.update(time_at(10));
        ASSERT_EQ(processing.estimates().size(), 1u);
        const auto & p = processing.estimates()[0].position;
        const auto error = std::sqrt(std::pow(p.x - truth.x, 2) + std::pow(p.y - truth.y, 2) + std::pow(p.z - truth.z, 2));
        sum_squared_error += error * error; max_error = std::max(max_error, error);
    }
    const auto rmse = std::sqrt(sum_squared_error / 100);
    RecordProperty("synthetic_rmse_metres", std::to_string(rmse));
    RecordProperty("synthetic_max_error_metres", std::to_string(max_error));
    EXPECT_LT(rmse, 0.03); EXPECT_LT(max_error, 0.06);
}
}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    const auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
