#include "mission_processing.h"

#include <cmath>
#include <limits>

MissionProcessing::MissionProcessing(rclcpp::Node & node)
{
	laser_scan_subscription_ = node.create_subscription<sensor_msgs::msg::LaserScan>(
		"/husky1/scan",
		rclcpp::SensorDataQoS(),
		[this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
			laser_scan_callback(std::move(scan));
		});
}

void MissionProcessing::laser_scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
	latest_laser_scan_ = std::move(scan);
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