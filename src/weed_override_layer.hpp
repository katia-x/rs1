#ifndef RS1_WEED_OVERRIDE_LAYER_HPP_
#define RS1_WEED_OVERRIDE_LAYER_HPP_

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>
#include <vector>

class WeedOverrideLayer : public nav2_costmap_2d::CostmapLayer
{
public:
    WeedOverrideLayer() = default;
    ~WeedOverrideLayer() override = default;

    void reset() override;
    bool isClearable() override { return false; }
    void updateBounds(double robot_x, double robot_y, double robot_yaw,
        double * min_x, double * min_y, double * max_x, double * max_y) override;
    void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
        int min_i, int min_j, int max_i, int max_j) override;

protected:
    void onInitialize() override;

private:
    struct WeedSnapshot
    {
        std::string frame_id;
        std::vector<geometry_msgs::msg::Point> positions;
    };

    std::vector<geometry_msgs::msg::Point> transformed_positions() const;
    void weed_callback(geometry_msgs::msg::PoseArray::ConstSharedPtr message);

    std::string weed_topic_;
    double override_radius_{0.35};
    std::string global_frame_;
    mutable std::mutex mutex_;
    WeedSnapshot snapshot_;
    std::vector<geometry_msgs::msg::Point> previous_positions_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

#endif  // RS1_WEED_OVERRIDE_LAYER_HPP_
