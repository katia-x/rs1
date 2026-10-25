#include "weed_override_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

PLUGINLIB_EXPORT_CLASS(WeedOverrideLayer, nav2_costmap_2d::Layer)

void WeedOverrideLayer::onInitialize()
{
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("weed_topic", rclcpp::ParameterValue(
        std::string("/husky1/permanent_weed_exclusions")));
    declareParameter("override_radius", rclcpp::ParameterValue(0.35));
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("Failed to lock costmap lifecycle node");
    }
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".weed_topic", weed_topic_);
    node->get_parameter(name_ + ".override_radius", override_radius_);
    global_frame_ = layered_costmap_->getGlobalFrameID();

    if (!std::isfinite(override_radius_) || override_radius_ <= 0.0) {
        throw std::invalid_argument("override_radius must be positive and finite");
    }

    subscription_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
        weed_topic_, rclcpp::QoS(1).transient_local(),
        std::bind(&WeedOverrideLayer::weed_callback, this, std::placeholders::_1));
    current_ = true;
}

void WeedOverrideLayer::weed_callback(geometry_msgs::msg::PoseArray::ConstSharedPtr message)
{
    WeedSnapshot next;
    next.frame_id = message->header.frame_id;
    next.positions.reserve(message->poses.size());
    for (const auto & pose : message->poses) {
        if (std::isfinite(pose.position.x) && std::isfinite(pose.position.y)) {
            next.positions.push_back(pose.position);
        }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = std::move(next);
    // The complete snapshot is already available to updateBounds/updateCosts.
    // Do not make Nav2 wait for another costmap cycle just because it changed.
    current_ = true;
}

std::vector<geometry_msgs::msg::Point> WeedOverrideLayer::transformed_positions() const
{
    WeedSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot = snapshot_;
    }
    if (snapshot.positions.empty()) {
        return {};
    }

    if (snapshot.frame_id.empty() || snapshot.frame_id == global_frame_) {
        return snapshot.positions;
    }

    try {
        const auto transform = tf_->lookupTransform(
            global_frame_, snapshot.frame_id, tf2::TimePointZero);
        std::vector<geometry_msgs::msg::Point> positions;
        positions.reserve(snapshot.positions.size());
        for (const auto & position : snapshot.positions) {
            geometry_msgs::msg::Point transformed;
            tf2::doTransform(position, transformed, transform);
            positions.push_back(transformed);
        }
        return positions;
    } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 5000,
            "Cannot transform weed exclusions from %s to %s: %s",
            snapshot.frame_id.c_str(), global_frame_.c_str(), error.what());
        return {};
    }
}

void WeedOverrideLayer::updateBounds(double, double, double,
    double * min_x, double * min_y, double * max_x, double * max_y)
{
    const auto positions = transformed_positions();
    const double radius = override_radius_;
    for (const auto & position : positions) {
        *min_x = std::min(*min_x, position.x - radius);
        *min_y = std::min(*min_y, position.y - radius);
        *max_x = std::max(*max_x, position.x + radius);
        *max_y = std::max(*max_y, position.y + radius);
    }
    for (const auto & position : previous_positions_) {
        *min_x = std::min(*min_x, position.x - radius);
        *min_y = std::min(*min_y, position.y - radius);
        *max_x = std::max(*max_x, position.x + radius);
        *max_y = std::max(*max_y, position.y + radius);
    }
    previous_positions_ = positions;
}

void WeedOverrideLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_) {
        return;
    }
    const auto positions = transformed_positions();
    const double radius_squared = override_radius_ * override_radius_;
    for (int y = min_j; y < max_j; ++y) {
        for (int x = min_i; x < max_i; ++x) {
            double world_x, world_y;
            master_grid.mapToWorld(static_cast<unsigned int>(x),
                static_cast<unsigned int>(y), world_x, world_y);
            const bool inside_weed = std::any_of(positions.begin(), positions.end(),
                [&](const auto & weed) {
                    const double dx = world_x - weed.x;
                    const double dy = world_y - weed.y;
                    return dx * dx + dy * dy <= radius_squared;
                });
            if (inside_weed) {
                master_grid.setCost(static_cast<unsigned int>(x),
                    static_cast<unsigned int>(y), nav2_costmap_2d::FREE_SPACE);
            }
        }
    }
    current_ = true;
}

void WeedOverrideLayer::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    previous_positions_.clear();
    current_ = false;
}
