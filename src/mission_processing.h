#ifndef RS1_MISSION_PROCESSING_H
#define RS1_MISSION_PROCESSING_H

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <vision_msgs/msg/detection2_d_array.hpp>

// Stateful perception model. MissionNode owns ROS subscriptions, scheduling,
// publishers and the shared TF listener. Calls are serialized by its executor.
class MissionProcessing
{
public:
    struct Config
    {
        std::string global_frame{"map"};
        double confidence_threshold{0.75};
        double max_time_difference{0.1};
        double max_observation_age{2.0};
        double roi_fraction{0.5};
        double depth_cluster_tolerance{0.15};
        int min_valid_samples{6};
        double min_cluster_fraction{0.6};
        double association_distance{0.5};
        double smoothing_factor{0.5};
        double track_timeout{3.0};
    };

    struct WeedEstimate
    {
        std::uint64_t id;
        geometry_msgs::msg::Point position;
        rclcpp::Time last_seen;
        std::size_t observation_count;
    };

    struct SearchLane
    {
        geometry_msgs::msg::Point start;
        geometry_msgs::msg::Point end;
        double half_width;  // Half the spacing between adjacent search lanes.
    };

    enum class UpdateResult
    {
        Idle, WaitingForCloud, WaitingForTransform, Rejected, Processed
    };

    struct CloudTiming
    {
        std::size_t buffered_clouds;
        std::size_t pending_batches;
        std::optional<rclcpp::Time> detection_stamp;
        std::optional<rclcpp::Time> nearest_cloud_stamp;
    };

    MissionProcessing(Config config, std::shared_ptr<tf2_ros::Buffer> tf_buffer);
    void receive_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
    void receive_detections(vision_msgs::msg::Detection2DArray::ConstSharedPtr detections);
    UpdateResult update(const rclcpp::Time & now);
    CloudTiming cloud_timing() const;

    const std::vector<WeedEstimate> & estimates() const noexcept { return estimates_; }
    const Config & config() const noexcept { return config_; }
    std::optional<WeedEstimate> select_target(
        const geometry_msgs::msg::Point & robot, const SearchLane & lane,
        const std::vector<geometry_msgs::msg::Pose> & region,
        std::optional<std::uint64_t> preferred_id = std::nullopt) const;

    // Legacy stateless image/lidar approach calculation; not used for navigation.
    static geometry_msgs::msg::Twist pinpointPlant(
        float xmin, float ymin, float xmax, float ymax,
        const sensor_msgs::msg::LaserScan & scan);

private:
    std::optional<geometry_msgs::msg::Point> estimate_point(
        const vision_msgs::msg::Detection2D & detection,
        const sensor_msgs::msg::PointCloud2 & cloud) const;
    void associate(const std::vector<geometry_msgs::msg::Point> & observations,
        const rclcpp::Time & stamp);
    void finish_batch();
    sensor_msgs::msg::PointCloud2::ConstSharedPtr nearest_cloud() const;

    Config config_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds_;
    std::deque<vision_msgs::msg::Detection2DArray::ConstSharedPtr> pending_detections_;
    std::optional<rclcpp::Time> last_processed_stamp_;
    std::optional<rclcpp::Time> last_update_time_;
    std::vector<WeedEstimate> estimates_;
    std::uint64_t next_id_{1};
};

#endif  // RS1_MISSION_PROCESSING_H
