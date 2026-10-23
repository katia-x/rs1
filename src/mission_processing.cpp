#include "mission_processing.h"

#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>
#include <functional>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{

constexpr double kWeedConfidenceThreshold = 0.75;
constexpr double kDuplicateDistance = 0.5;
constexpr std::size_t kPointCloudHistorySize = 30;
constexpr char kGlobalFrame[] = "map";

double timestamp_difference(const builtin_interfaces::msg::Time & first,
    const builtin_interfaces::msg::Time & second)
{
    return std::abs((rclcpp::Time(first) - rclcpp::Time(second)).seconds());
}

}

MissionProcessing::MissionProcessing(rclcpp::Node & node)
    : tf_buffer_(node.get_clock()),
      tf_listener_(tf_buffer_)
{
	laser_scan_subscription_ = node.create_subscription<sensor_msgs::msg::LaserScan>(
		"/husky1/scan",
		rclcpp::SensorDataQoS(),
		[this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
			laser_scan_callback(std::move(scan));
		});

    bounding_box_subscription_ = node.create_subscription<vision_msgs::msg::Detection2DArray>(
		"/husky1/detections",
		rclcpp::SensorDataQoS(),
        [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr detections) {
            bounding_box_callback(std::move(detections));
		});

    point_cloud_subscription_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
        "/husky1/camera/depth/points",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud) {
            depth_point_cloud_callback(std::move(point_cloud));
        });

    weed_check_timer_ = node.create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MissionProcessing::check_for_weeds, this));
}

void MissionProcessing::depth_point_cloud_callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud)
{
    point_cloud_history_.push_back(std::move(point_cloud));
    if (point_cloud_history_.size() > kPointCloudHistorySize) {
        point_cloud_history_.pop_front();
    }
}

void MissionProcessing::check_for_weeds()
{
    if (!latest_bounding_boxes_ || point_cloud_history_.empty()) {
        return;
    }

    const auto detection_stamp = latest_bounding_boxes_->header.stamp;
    const auto point_cloud = *std::min_element(
        point_cloud_history_.begin(), point_cloud_history_.end(),
        [&detection_stamp](const auto & first, const auto & second) {
            return timestamp_difference(first->header.stamp, detection_stamp) <
                timestamp_difference(second->header.stamp, detection_stamp);
        });

    try {
        const auto transform = tf_buffer_.lookupTransform(
            kGlobalFrame,
            point_cloud->header.frame_id,
            rclcpp::Time(point_cloud->header.stamp));

        for (const auto & detection : latest_bounding_boxes_->detections) {
            for (const auto & result : detection.results) {
                if (result.hypothesis.class_id != "Weed" ||
                    result.hypothesis.score <= kWeedConfidenceThreshold) {
                    continue;
                }

                if (point_cloud->width == 0 || point_cloud->height == 0) {
                    return;
                }
                const auto & bounding_box = detection.bbox;

                const auto min_x = std::clamp(
                    std::lround(bounding_box.center.position.x - bounding_box.size_x / 2.0),
                    0L, static_cast<long>(point_cloud->width - 1));
                const auto max_x = std::clamp(
                    std::lround(bounding_box.center.position.x + bounding_box.size_x / 2.0),
                    0L, static_cast<long>(point_cloud->width - 1));
                const auto min_y = std::clamp(
                    std::lround(bounding_box.center.position.y - bounding_box.size_y / 2.0),
                    0L, static_cast<long>(point_cloud->height - 1));
                const auto max_y = std::clamp(
                    std::lround(bounding_box.center.position.y + bounding_box.size_y / 2.0),
                    0L, static_cast<long>(point_cloud->height - 1));

                double sum_x = 0.0;
                double sum_y = 0.0;
                double sum_z = 0.0;
                std::size_t valid_sample_count = 0;
                for (int sample_y = 0; sample_y < 3; ++sample_y) {
                    const auto pixel_y = static_cast<std::size_t>(
                        min_y + (max_y - min_y) * sample_y / 2);
                    for (int sample_x = 0; sample_x < 3; ++sample_x) {
                        const auto pixel_x = static_cast<std::size_t>(
                            min_x + (max_x - min_x) * sample_x / 2);
                        const auto point_index = pixel_y * point_cloud->width + pixel_x;

                        sensor_msgs::PointCloud2ConstIterator<float> x(*point_cloud, "x");
                        sensor_msgs::PointCloud2ConstIterator<float> y(*point_cloud, "y");
                        sensor_msgs::PointCloud2ConstIterator<float> z(*point_cloud, "z");
                        x += point_index;
                        y += point_index;
                        z += point_index;
                        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
                            continue;
                        }

                        sum_x += *x;
                        sum_y += *y;
                        sum_z += *z;
                        ++valid_sample_count;
                    }
                }
                if (valid_sample_count == 0) {
                    continue;
                }

                geometry_msgs::msg::PointStamped camera_point;
                camera_point.header = point_cloud->header;
                camera_point.point.x = sum_x / valid_sample_count;
                camera_point.point.y = sum_y / valid_sample_count;
                camera_point.point.z = sum_z / valid_sample_count;
                geometry_msgs::msg::PointStamped global_point;
                tf2::doTransform(camera_point, global_point, transform);

                const auto duplicate = std::any_of(
                    stored_weed_coordinates_.begin(), stored_weed_coordinates_.end(),
                    [&global_point](const auto & stored_point) {
                        const double dx = stored_point.x - global_point.point.x;
                        const double dy = stored_point.y - global_point.point.y;
                        const double dz = stored_point.z - global_point.point.z;
                        return std::sqrt(dx * dx + dy * dy + dz * dz) < kDuplicateDistance;
                    });
                if (!duplicate) {
                    stored_weed_coordinates_.push_back(global_point.point);
                }
            }
        }
    } catch (const tf2::TransformException &) {
        return;
    } catch (const std::runtime_error &) {
        return;
    }
}

void MissionProcessing::laser_scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
	latest_laser_scan_ = std::move(scan);
}

void MissionProcessing::bounding_box_callback(
    vision_msgs::msg::Detection2DArray::ConstSharedPtr detections)
{
    latest_bounding_boxes_ = std::move(detections);
}

// Converts bounding box coordinates to a velocity command. 
// The bounding box is defined by the minimum and maximum x and y coordinates temporarily.
// Needs to be modified once the ML model is trained.
geometry_msgs::msg::Twist MissionProcessing::pinpointPlant(
    float xmin, float ymin, float xmax, float ymax,
    const sensor_msgs::msg::LaserScan & scan)
{
    geometry_msgs::msg::Twist cmd;  // Create empty velocity command message to be filled.

    // -- Constants for the algorithm -- double check virtual cam actually has these parameters
    constexpr double pi = M_PI;
    constexpr double image_width = 720.0;
    constexpr double image_height = 480.0;
    constexpr double horizontal_fov = 2.0944;  // Supplied camera Xacro, radians.
    const double fx = image_width / (2.0 * std::tan(horizontal_fov / 2.0));
    const double cx = image_width / 2.0;
    constexpr double camera_x = 0.45;  // Camera's forward offset from base_link.
    constexpr double match_window = 1.5 * pi / 180.0;
    constexpr double alignment_tolerance = 2.0 * pi / 180.0;

    // -- Constants defining WOMBAT limits --
    constexpr double stop_distance = 0.8; // Range from lidar/base XY origin.
    constexpr double turn_gain = 1.0;
    constexpr double max_turn_speed = 0.5; // rad/s
    constexpr double drive_gain = 0.3;
    constexpr double max_drive_speed = 0.3; // m/s

    // Confirm arguments are valid and within the image bounds. If not, return a zero command.
    if (!std::isfinite(xmin) || !std::isfinite(ymin) || !std::isfinite(xmax) || !std::isfinite(ymax) ||
        xmin < 0 || ymin < 0 || xmax > image_width || ymax > image_height || xmin >= xmax || ymin >= ymax)
        return cmd;

    // Normalise the angle
    const auto wrap = [](double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    };

    // Calculate the center of the bounding box.
    const double u = (static_cast<double>(xmin) + xmax) / 2.0;

    // Calculate the bearing to the center of the bounding box in the camera frame.
    const double image_bearing = std::atan2(cx - u, fx);
    double robot_bearing = image_bearing; // Fallback: turn even without lidar.
    double target_range = std::numeric_limits<double>::infinity();

    // Check if the lidar scan is valid.
    const bool valid_scan = std::isfinite(scan.angle_min) &&
        std::isfinite(scan.angle_increment) && scan.angle_increment != 0.0 &&
        std::isfinite(scan.range_min) && std::isfinite(scan.range_max) &&
        scan.range_min >= 0.0 && scan.range_max > scan.range_min;

    // Iterate through the lidar scan to find the nearest return within the bounding box
    if (valid_scan) {
        for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
            const double r = scan.ranges[i];
            if (!std::isfinite(r) || r <= 0.0 ||
                r < scan.range_min || r > scan.range_max)
                continue;

            const double angle = scan.angle_min + i * scan.angle_increment;
            const double x = r * std::cos(angle);
            const double y = r * std::sin(angle);
            if (x <= camera_x) continue; // Behind the camera's image plane.

            // Compare each return's direction FROM THE CAMERA to the pixel ray.
            // This compensates for the 0.45 m camera/lidar forward separation.
            const double bearing_from_camera = std::atan2(y, x - camera_x);
            if (std::abs(wrap(bearing_from_camera - image_bearing)) > match_window)
                continue;

            // If the return is within the bounding box, check if it's the closest one.
            if (r < target_range) {
                target_range = r;
                robot_bearing = std::atan2(y, x);
            }
        }
    }

	// Plant detected to the side of the WOMBAT -- turn to face it first
    if (std::abs(robot_bearing) > alignment_tolerance) {
        cmd.angular.z = std::max(-max_turn_speed,
            std::min(max_turn_speed, turn_gain * robot_bearing));
        return cmd; 
    }

    // WOMBAT facing plant -- Drive forward towards it
    if (std::isfinite(target_range) && target_range > stop_distance) {
        cmd.linear.x = std::min(max_drive_speed,
            drive_gain * (target_range - stop_distance));
    }
	
    return cmd;
}