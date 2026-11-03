#ifndef RS1_MISSION_PROCESSING_H
#define RS1_MISSION_PROCESSING_H

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <map>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <vision_msgs/msg/detection2_d_array.hpp>

// Stateful perception model. MissionNode owns ROS subscriptions, scheduling,
// publishers and the shared TF listener. Calls are serialized by its executor.
class MissionProcessing
{
public:
    // Colour and geometry thresholds are simulation heuristics, not semantic labels.
    struct RockConfig
    {
        bool enabled{false};
        int max_saturation{70};           // OpenCV HSV: 0..255.
        int min_brightness{35};
        int max_brightness{235};
        double ground_z{0.0};             // Flat simulated ground, in global_frame.
        double min_height{0.08};          // Metres above that ground.
        double max_height{2.0};
        double max_range{8.0};            // Euclidean camera range, metres.
        double neighbour_distance{0.12};  // Maximum 3D gap between adjacent pixels.
        int min_component_pixels{40};
        double min_horizontal_extent{0.15};
        double cell_size{0.05};           // Sparse debug-map resolution, metres.
        double max_time_difference{0.03};
        std::size_t max_map_cells{100000};
    };

    struct RockFrame
    {
        std_msgs::msg::Header header;  // RGB acquisition header.
        cv::Mat mask;                 // CV_8UC1: accepted rock pixels, 0 or 255.
        std::vector<geometry_msgs::msg::Point> points;  // In global_frame.
    };

    struct Config
    {
        std::string global_frame{"map"};
        RockConfig rock;
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

    void receive_image(sensor_msgs::msg::Image::ConstSharedPtr image);
    std::optional<RockFrame> update_rocks(const rclcpp::Time & now);
    // Stateless operation exposed so geometry/colour decisions can be tested offline.
    RockFrame detect_rocks(const cv::Mat & bgr,
        const sensor_msgs::msg::PointCloud2 & cloud,
        const geometry_msgs::msg::TransformStamped & cloud_to_global) const;
    const std::map<std::pair<int, int>, geometry_msgs::msg::Point> & rock_cells() const noexcept
    {
        return rock_cells_;
    }

    const std::vector<WeedEstimate> & estimates() const noexcept
    {
        return estimates_;
    }

    const std::vector<WeedEstimate> & permanent_weeds() const noexcept
    {
        return permanent_weeds_;
    }

    const Config & config() const noexcept
    {
        return config_;
    }

    std::optional<WeedEstimate> select_target(
        const geometry_msgs::msg::Point & robot, const SearchLane & lane,
        const std::vector<geometry_msgs::msg::Pose> & region,
        std::optional<std::uint64_t> preferred_id = std::nullopt,
        const std::unordered_set<std::uint64_t> & excluded_ids = {}) const;

private:
    std::optional<geometry_msgs::msg::Point> estimate_point(
        const vision_msgs::msg::Detection2D & detection,
        const sensor_msgs::msg::PointCloud2 & cloud) const;
    void associate(const std::vector<geometry_msgs::msg::Point> & observations,
        const rclcpp::Time & stamp);
    void remember_permanent_weed(const WeedEstimate & estimate);
    void finish_batch();
    sensor_msgs::msg::PointCloud2::ConstSharedPtr nearest_cloud() const;

    Config config_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds_;
    std::deque<vision_msgs::msg::Detection2DArray::ConstSharedPtr> pending_detections_;
    std::optional<rclcpp::Time> last_processed_stamp_;
    std::optional<rclcpp::Time> last_update_time_;
    std::vector<WeedEstimate> estimates_;
    std::vector<WeedEstimate> permanent_weeds_;
    std::uint64_t next_id_{1};
    std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images_;
    std::optional<rclcpp::Time> last_rock_stamp_;
    std::optional<rclcpp::Time> last_rock_update_time_;
    std::map<std::pair<int, int>, geometry_msgs::msg::Point> rock_cells_;

};

#endif  // RS1_MISSION_PROCESSING_H
