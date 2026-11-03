#include "mission_processing.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
constexpr std::size_t kCloudHistorySize = 30;
constexpr std::size_t kPendingDetectionLimit = 30;
// constexpr int kSampleGridSize = 7;

bool finite_point(const geometry_msgs::msg::Point & point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double squared_distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

// Reads organized FLOAT32 XYZ clouds, including row padding and either byte order.
class CloudReader
{
public:
    explicit CloudReader(const sensor_msgs::msg::PointCloud2 & cloud) : cloud_(cloud)
    {
        if (cloud.width < 2 || cloud.height < 2 || cloud.point_step == 0 ||
            static_cast<std::uint64_t>(cloud.row_step) <
            static_cast<std::uint64_t>(cloud.width) * cloud.point_step ||
            static_cast<std::uint64_t>(cloud.row_step) * cloud.height > cloud.data.size()) {
            throw std::invalid_argument("Depth cloud must have a valid organized layout");
        }
        const std::array<std::string, 3> names{{"x", "y", "z"}};
        for (std::size_t i = 0; i < names.size(); ++i) {
            const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                [&](const auto & f) {
                    return f.name == names[i];
                });
            if (field == cloud.fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
                field->count != 1 || static_cast<std::uint64_t>(field->offset) + sizeof(float) > cloud.point_step) {
                throw std::invalid_argument("Depth cloud requires FLOAT32 x, y and z fields");
            }
            offsets_[i] = field->offset;
        }
    }

    geometry_msgs::msg::Point at(std::uint32_t u, std::uint32_t v) const
    {
        const auto base = static_cast<std::size_t>(v) * cloud_.row_step +
            static_cast<std::size_t>(u) * cloud_.point_step;
        std::array<float, 3> xyz;
        const std::uint16_t endian_probe = 1;
        const bool host_bigendian = *reinterpret_cast<const std::uint8_t *>(&endian_probe) == 0;
        for (std::size_t i = 0; i < xyz.size(); ++i) {
            std::array<std::uint8_t, sizeof(float)> bytes;
            std::memcpy(bytes.data(), cloud_.data.data() + base + offsets_[i], sizeof(float));
            if (cloud_.is_bigendian != host_bigendian) {
                std::reverse(bytes.begin(), bytes.end());
            }
            std::memcpy(&xyz[i], bytes.data(), sizeof(float));
        }
        geometry_msgs::msg::Point point;
        point.x = xyz[0];
        point.y = xyz[1];
        point.z = xyz[2];
        return point;
    }

private:
    const sensor_msgs::msg::PointCloud2 & cloud_;
    std::array<std::uint32_t, 3> offsets_{};
};

bool inside_region(const geometry_msgs::msg::Point & point,
    const std::vector<geometry_msgs::msg::Pose> & region)
{
    if (region.size() < 3) {
        return false;
    }
    bool inside = false;
    for (std::size_t i = 0, j = region.size() - 1; i < region.size(); j = i++) {
        const auto & a = region[j].position;
        const auto & b = region[i].position;
        if (!finite_point(a) || !finite_point(b)) {
            return false;
        }
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double cross = (point.x - a.x) * dy - (point.y - a.y) * dx;
        if (std::abs(cross) < 1e-9 && point.x >= std::min(a.x, b.x) - 1e-9 &&
            point.x <= std::max(a.x, b.x) + 1e-9 && point.y >= std::min(a.y, b.y) - 1e-9 &&
            point.y <= std::max(a.y, b.y) + 1e-9) {
            return true;
        }
        if ((a.y > point.y) != (b.y > point.y) &&
            point.x < a.x + (point.y - a.y) * dx / dy) {
            inside = !inside;
        }
    }
    return inside;
}

}  // namespace

MissionProcessing::MissionProcessing(Config config, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : config_(std::move(config)), tf_buffer_(std::move(tf_buffer))
{
    const auto positive = [](double v) {
        return std::isfinite(v) && v > 0.0;
    };
    const auto & rock = config_.rock;
    if (rock.max_saturation < 0 || rock.max_saturation > 255 ||
        rock.min_brightness < 0 || rock.max_brightness > 255 ||
        rock.min_brightness > rock.max_brightness || !std::isfinite(rock.ground_z) ||
        !positive(rock.min_height) || !positive(rock.max_height) ||
        rock.min_height >= rock.max_height || !positive(rock.max_range) ||
        !positive(rock.neighbour_distance) || rock.min_component_pixels < 1 ||
        !positive(rock.min_horizontal_extent) || !positive(rock.cell_size) ||
        !positive(rock.max_time_difference) || rock.max_map_cells == 0) {
        throw std::invalid_argument("Invalid rock detection configuration");
    }
    if (!tf_buffer_ || config_.global_frame.empty() ||
        !std::isfinite(config_.confidence_threshold) || config_.confidence_threshold < 0.0 ||
        config_.confidence_threshold > 1.0 || !positive(config_.max_time_difference) ||
        !positive(config_.max_observation_age) || !positive(config_.roi_fraction) || config_.roi_fraction > 1.0 ||
        !positive(config_.depth_cluster_tolerance) || config_.min_valid_samples < 3 ||
        // config_.min_valid_samples > kSampleGridSize * kSampleGridSize ||
        !positive(config_.min_cluster_fraction) || config_.min_cluster_fraction <= 0.0 ||
        config_.min_cluster_fraction > 1.0 || !positive(config_.association_distance) ||
        !positive(config_.smoothing_factor) || config_.smoothing_factor > 1.0 ||
        !positive(config_.track_timeout)) {
        throw std::invalid_argument("Invalid weed estimation configuration");
    }
}

void MissionProcessing::receive_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
    if (!cloud || cloud->header.stamp.sec < 0 || cloud->header.stamp.nanosec >= 1000000000u) {
        return;
    }
    clouds_.push_back(std::move(cloud));
    if (clouds_.size() > kCloudHistorySize) {
        clouds_.pop_front();
    }
}

void MissionProcessing::receive_detections(vision_msgs::msg::Detection2DArray::ConstSharedPtr detections)
{
    if (!detections || detections->header.stamp.sec < 0 ||
        detections->header.stamp.nanosec >= 1000000000u) {
        return;
    }
    const rclcpp::Time stamp(detections->header.stamp);
    if (last_processed_stamp_ && stamp <= *last_processed_stamp_) {
        return;
    }
    const auto position = std::lower_bound(pending_detections_.begin(), pending_detections_.end(), stamp,
        [](const auto & batch, const rclcpp::Time & time) {
            return rclcpp::Time(batch->header.stamp) < time;
        });
    if (position != pending_detections_.end() && rclcpp::Time((*position)->header.stamp) == stamp) {
        return;
    }
    // Preserve earlier detections while their larger depth messages are still
    // in flight. Replacing them with the newest inference result can starve
    // matching indefinitely when the depth stream has a consistent delay.
    pending_detections_.insert(position, std::move(detections));
    if (pending_detections_.size() > kPendingDetectionLimit) {
        finish_batch();
    }
}

void MissionProcessing::finish_batch()
{
    last_processed_stamp_ = rclcpp::Time(pending_detections_.front()->header.stamp);
    pending_detections_.pop_front();
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr MissionProcessing::nearest_cloud() const
{
    if (pending_detections_.empty() || clouds_.empty()) {
        return nullptr;
    }
    const rclcpp::Time stamp(pending_detections_.front()->header.stamp);
    return *std::min_element(clouds_.begin(), clouds_.end(), [&](const auto & a, const auto & b) {
                   return std::abs((rclcpp::Time(a->header.stamp) - stamp).nanoseconds()) <
                   std::abs((rclcpp::Time(b->header.stamp) - stamp).nanoseconds());
               });
}

MissionProcessing::CloudTiming MissionProcessing::cloud_timing() const
{
    CloudTiming timing{clouds_.size(), pending_detections_.size(), std::nullopt, std::nullopt};
    if (!pending_detections_.empty()) {
        timing.detection_stamp = rclcpp::Time(pending_detections_.front()->header.stamp);
    }
    const auto cloud = nearest_cloud();
    if (cloud) {
        timing.nearest_cloud_stamp = rclcpp::Time(cloud->header.stamp);
    }
    return timing;
}

MissionProcessing::UpdateResult MissionProcessing::update(const rclcpp::Time & now)
{
    if (last_update_time_ && now < *last_update_time_) {
        // Simulation resets invalidate timestamps, buffered clouds and map tracks.
        clouds_.clear();
        pending_detections_.clear();
        estimates_.clear();
        last_processed_stamp_.reset();
    }
    last_update_time_ = now;
    estimates_.erase(std::remove_if(estimates_.begin(), estimates_.end(), [&](const auto & track) {
            return (now - track.last_seen).seconds() > config_.track_timeout;
        }), estimates_.end());
    if (pending_detections_.empty()) {
        return UpdateResult::Idle;
    }
    const rclcpp::Time detection_stamp(pending_detections_.front()->header.stamp);
    if (detection_stamp > now) {
        return UpdateResult::Idle;
    }
    if ((now - detection_stamp).seconds() > config_.max_observation_age) {
        finish_batch();
        return UpdateResult::Rejected;
    }
    if (pending_detections_.front()->detections.empty()) {
        finish_batch();
        return UpdateResult::Processed;
    }
    const auto closest = nearest_cloud();
    if (!closest) {
        return UpdateResult::WaitingForCloud;
    }
    const auto & cloud = *closest;
    const rclcpp::Time cloud_stamp(cloud.header.stamp);
    if (std::abs((cloud_stamp - detection_stamp).seconds()) > config_.max_time_difference || cloud_stamp > now) {
        return UpdateResult::WaitingForCloud;
    }
    if (cloud.header.frame_id.empty() || cloud.header.frame_id != pending_detections_.front()->header.frame_id ||
        (now - cloud_stamp).seconds() > config_.max_observation_age || cloud_stamp.nanoseconds() == 0) {
        finish_batch();
        return UpdateResult::Rejected;
    }
    geometry_msgs::msg::TransformStamped transform;
    try {
        // Never replace an unavailable acquisition-time transform with the latest TF.
        transform = tf_buffer_->lookupTransform(
            config_.global_frame, cloud.header.frame_id, tf2_ros::fromRclcpp(cloud_stamp));
    } catch (const tf2::TransformException &) {
        return UpdateResult::WaitingForTransform;
    }
    std::vector<geometry_msgs::msg::Point> observations;
    try {
        const CloudReader validate_layout(cloud);
        for (const auto & detection : pending_detections_.front()->detections) {
            const bool weed = std::any_of(detection.results.begin(), detection.results.end(), [&](const auto & result) {
                    return result.hypothesis.class_id == "Weed" && std::isfinite(result.hypothesis.score) &&
                    result.hypothesis.score > config_.confidence_threshold;
                });
            if (!weed) {
                continue;
            }
            const auto point = estimate_point(detection, cloud);
            if (!point) {
                continue;
            }
            geometry_msgs::msg::PointStamped camera_point, map_point;
            camera_point.header = cloud.header;
            camera_point.point = *point;
            tf2::doTransform(camera_point, map_point, transform);
            if (finite_point(map_point.point)) {
                observations.push_back(map_point.point);
            }
        }
    } catch (const std::invalid_argument &) {
        finish_batch();
        return UpdateResult::Rejected;
    }
    associate(observations, cloud_stamp);
    finish_batch();
    return UpdateResult::Processed;
}

std::optional<geometry_msgs::msg::Point> MissionProcessing::estimate_point(
    const vision_msgs::msg::Detection2D & detection, const sensor_msgs::msg::PointCloud2 & cloud) const
{
    const auto & box = detection.bbox;
    const double cx = box.center.position.x, cy = box.center.position.y;

    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(box.size_x) || !std::isfinite(box.size_y) ||
        box.size_x <= 0.0 || box.size_y <= 0.0 || !std::isfinite(box.center.theta) ||
        std::abs(box.center.theta) > 1e-6) {
        return std::nullopt;
    }

    const double half_w = box.size_x * config_.roi_fraction / 2.0;
    const double half_h = box.size_y * config_.roi_fraction / 2.0;
    const double left = std::max(0.0, cx - half_w), right = std::min(double(cloud.width - 1), cx + half_w);
    const double top = std::max(0.0, cy - half_h), bottom = std::min(double(cloud.height - 1), cy + half_h);

    if (left > right || top > bottom) {
        return std::nullopt;
    }
    const CloudReader reader(cloud);
    std::vector<geometry_msgs::msg::Point> samples;

    const int min_u = static_cast<int>(std::ceil(left));
    const int max_u = static_cast<int>(std::floor(right));
    const int min_v = static_cast<int>(std::ceil(top));
    const int max_v = static_cast<int>(std::floor(bottom));

    for (int v = min_v; v <= max_v; ++v) {
        for (int u = min_u; u <= max_u; ++u) {
            const auto point = reader.at(u, v);
            if (finite_point(point) && point.x > 0.0) {
                samples.push_back(point);
            }
        }
    }

    // std::vector<std::uint64_t> pixels;

    // for (int row = 0; row < kSampleGridSize; ++row) {
    //     const auto v = static_cast<std::uint32_t>(std::lround(top + (bottom - top) * row / (kSampleGridSize - 1)));
    //     for (int col = 0; col < kSampleGridSize; ++col) {
    //         const auto u = static_cast<std::uint32_t>(std::lround(left + (right - left) * col / (kSampleGridSize - 1)));
    //         const std::uint64_t pixel = static_cast<std::uint64_t>(v) * cloud.width + u;
    //         if (std::find(pixels.begin(), pixels.end(), pixel) != pixels.end()) {
    //             continue;
    //         }
    //         pixels.push_back(pixel);
    //         const auto point = reader.at(u, v);
    //         // This registered Fortress cloud is expressed in camera_link: X forward.
    //         if (finite_point(point) && point.x > 0.0) {
    //             samples.push_back(point);
    //         }
    //     }
    // }

    if (samples.size() < static_cast<std::size_t>(config_.min_valid_samples)) {
        return std::nullopt;
    }
    std::sort(samples.begin(), samples.end(), [](const auto & a, const auto & b) {
            return a.x < b.x;
        });
    std::size_t best_begin = 0, best_end = 0;
    for (std::size_t begin = 0, end = 0; begin < samples.size(); ++begin) {
        while (end < samples.size() && samples[end].x - samples[begin].x <= config_.depth_cluster_tolerance) {
            ++end;
        }
        if (end - begin > best_end - best_begin) {
            best_begin = begin;
            best_end = end;
        }
    }
    const auto count = best_end - best_begin;
    if (count < static_cast<std::size_t>(config_.min_valid_samples) ||
        double(count) / samples.size() < config_.min_cluster_fraction) {
        return std::nullopt;
    }
    // Find a robust centre, then return an actual observed point nearest it.
    geometry_msgs::msg::Point centre;
    for (int axis = 0; axis < 3; ++axis) {
        std::vector<double> values;
        for (auto i = best_begin; i < best_end; ++i) {
            values.push_back(axis == 0 ? samples[i].x : axis == 1 ? samples[i].y : samples[i].z);
        }
        std::sort(values.begin(), values.end());
        const double median = (values[(count - 1) / 2] + values[count / 2]) / 2.0;
        if (axis == 0) {
            centre.x = median;
        } else if (axis == 1) {
            centre.y = median;
        } else {
            centre.z = median;
        }
    }
    return *std::min_element(samples.begin() + best_begin, samples.begin() + best_end,
               [&](const auto & a, const auto & b) {
                   return squared_distance(a, centre) < squared_distance(b, centre);
               });
}

void MissionProcessing::associate(const std::vector<geometry_msgs::msg::Point> & observations,
    const rclcpp::Time & stamp)
{
    struct Match {
        double distance;
        std::size_t observation;
        std::size_t track;
    };
    std::vector<Match> matches;
    const auto old_size = estimates_.size();
    for (std::size_t i = 0; i < observations.size(); ++i) {
        for (std::size_t j = 0; j < old_size; ++j) {
            const auto d = squared_distance(observations[i], estimates_[j].position);
            if (d <= config_.association_distance * config_.association_distance) {
                matches.push_back({d, i, j});
            }
        }
    }
    std::sort(matches.begin(), matches.end(), [](const auto & a, const auto & b) {
            return a.distance < b.distance;
        });
    std::vector<bool> used_observations(observations.size()), used_tracks(old_size);
    for (const auto & match : matches) {
        if (used_observations[match.observation] || used_tracks[match.track]) {
            continue;
        }
        used_observations[match.observation] = true;
        used_tracks[match.track] = true;
        auto & track = estimates_[match.track];
        if (stamp <= track.last_seen) {
            continue;
        }
        const auto & point = observations[match.observation];
        const auto alpha = config_.smoothing_factor;
        track.position.x += alpha * (point.x - track.position.x);
        track.position.y += alpha * (point.y - track.position.y);
        track.position.z += alpha * (point.z - track.position.z);
        track.last_seen = stamp;
        ++track.observation_count;
        remember_permanent_weed(track);
    }
    for (std::size_t i = 0; i < observations.size(); ++i) {
        if (!used_observations[i]) {
            estimates_.push_back({next_id_++, observations[i], stamp, 1});
            remember_permanent_weed(estimates_.back());
        }
    }
}

void MissionProcessing::remember_permanent_weed(const WeedEstimate & estimate)
{
    const auto existing = std::find_if(permanent_weeds_.begin(), permanent_weeds_.end(),
        [&](const auto & weed) {
            return weed.id == estimate.id;
        });
    if (existing == permanent_weeds_.end()) {
        permanent_weeds_.push_back(estimate);
        return;
    }
    existing->position = estimate.position;
    existing->last_seen = estimate.last_seen;
    existing->observation_count = estimate.observation_count;
}

std::optional<MissionProcessing::WeedEstimate> MissionProcessing::select_target(
    const geometry_msgs::msg::Point & robot, const SearchLane & lane,
    const std::vector<geometry_msgs::msg::Pose> & region, std::optional<std::uint64_t> preferred_id,
    const std::unordered_set<std::uint64_t> & excluded_ids) const
{
    if (!finite_point(robot) || !finite_point(lane.start) || !finite_point(lane.end) ||
        !std::isfinite(lane.half_width) || lane.half_width <= 0.0) {
        return std::nullopt;
    }
    const auto dx = lane.end.x - lane.start.x, dy = lane.end.y - lane.start.y;
    const auto length = std::hypot(dx, dy);
    if (length < 1e-6) {
        return std::nullopt;
    }
    const auto ux = dx / length, uy = dy / length;
    const auto progress = (robot.x - lane.start.x) * ux + (robot.y - lane.start.y) * uy;
    const WeedEstimate * selected = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto & track : estimates_) {
        if (excluded_ids.find(track.id) != excluded_ids.end()) {
            continue;
        }
        const auto x = track.position.x - lane.start.x, y = track.position.y - lane.start.y;
        const auto along = x * ux + y * uy;
        const auto across = std::abs(x * uy - y * ux);
        if (along < 0.0 || along <= progress || along > length || across > lane.half_width ||
            !inside_region(track.position, region)) {
            continue;
        }
        if (preferred_id && track.id == *preferred_id) {
            return track;
        }
        const auto distance = std::hypot(track.position.x - robot.x, track.position.y - robot.y);
        if (distance < best_distance) {
            selected = &track;
            best_distance = distance;
        }
    }
    return selected ? std::optional<WeedEstimate>(*selected) : std::nullopt;
}

void MissionProcessing::receive_image(sensor_msgs::msg::Image::ConstSharedPtr image)
{
    if (!config_.rock.enabled || !image || image->header.stamp.sec < 0 ||
        image->header.stamp.nanosec >= 1000000000u) {
        return;
    }
    const rclcpp::Time stamp(image->header.stamp);
    if (last_rock_stamp_ && stamp <= *last_rock_stamp_) {
        return;
    }
    const auto position = std::lower_bound(images_.begin(), images_.end(), stamp,
        [](const auto & frame, const rclcpp::Time & time) {
            return rclcpp::Time(frame->header.stamp) < time;
        });
    if (position == images_.end() || rclcpp::Time((*position)->header.stamp) != stamp) {
        images_.insert(position, std::move(image));
    }
    if (images_.size() > 10) {
        images_.pop_front();
    }
}

std::optional<MissionProcessing::RockFrame> MissionProcessing::update_rocks(const rclcpp::Time & now)
{
    if (!config_.rock.enabled) {
        return std::nullopt;
    }
    if (last_rock_update_time_ && now < *last_rock_update_time_) {
        images_.clear();
        rock_cells_.clear();
        last_rock_stamp_.reset();
        last_rock_update_time_ = now;
        return std::nullopt;
    }
    last_rock_update_time_ = now;
    while (!images_.empty()) {
        const auto image = images_.front();
        const rclcpp::Time stamp(image->header.stamp);
        if (stamp > now) {
            return std::nullopt;
        }
        if (stamp.nanoseconds() == 0 || (now - stamp).seconds() > config_.max_observation_age) {
            images_.pop_front();
            continue;
        }
        if (clouds_.empty()) {
            return std::nullopt;
        }
        const auto nearest = std::min_element(clouds_.begin(), clouds_.end(),
            [&](const auto & a, const auto & b) {
                return std::abs((rclcpp::Time(a->header.stamp) - stamp).nanoseconds()) <
                std::abs((rclcpp::Time(b->header.stamp) - stamp).nanoseconds());
            });
        const auto cloud = *nearest;
        const rclcpp::Time cloud_stamp(cloud->header.stamp);
        if (cloud_stamp > now) {
            return std::nullopt;
        }
        if (std::abs((cloud_stamp - stamp).seconds()) > config_.rock.max_time_difference) {
            const bool newer_cloud_exists = std::any_of(clouds_.begin(), clouds_.end(),
                [&](const auto & candidate) {
                    return (rclcpp::Time(candidate->header.stamp) - stamp).seconds() >
                    config_.rock.max_time_difference;
                });
            if (newer_cloud_exists) {
                images_.pop_front();
                continue;
            }
            return std::nullopt;
        }
        // This assumes the Gazebo RGB and organized cloud are registered.
        // Equal dimensions/frame IDs alone do not establish registration for other cameras.
        if (image->header.frame_id.empty() || image->header.frame_id != cloud->header.frame_id) {
            images_.pop_front();
            throw std::invalid_argument("Rock RGB/cloud frame IDs differ or are empty");
        }
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                config_.global_frame, cloud->header.frame_id,
                tf2_ros::fromRclcpp(cloud_stamp));
        } catch (const tf2::TransformException &) {
            return std::nullopt;  // Keep the frame until TF arrives or it becomes stale.
        }
        images_.pop_front();
        last_rock_stamp_ = stamp;
        const auto colour = cv_bridge::toCvShare(image, "bgr8");
        auto result = detect_rocks(colour->image, *cloud, transform);
        result.header = image->header;
        const double cell_size = config_.rock.cell_size;
        for (const auto & point : result.points) {
            const double x = std::floor(point.x / cell_size);
            const double y = std::floor(point.y / cell_size);
            if (x < std::numeric_limits<int>::min() || x > std::numeric_limits<int>::max() ||
                y < std::numeric_limits<int>::min() || y > std::numeric_limits<int>::max()) {
                continue;
            }
            const auto key = std::make_pair(static_cast<int>(x), static_cast<int>(y));
            auto existing = rock_cells_.find(key);
            if (existing == rock_cells_.end()) {
                if (rock_cells_.size() >= config_.rock.max_map_cells) {
                    throw std::runtime_error("Rock debug map reached max_map_cells; reset before continuing");
                }
                geometry_msgs::msg::Point centre = point;
                centre.x = (x + 0.5) * cell_size;
                centre.y = (y + 0.5) * cell_size;
                rock_cells_.emplace(key, centre);
            } else {
                existing->second.z = std::max(existing->second.z, point.z);
            }
        }
        return result;
    }
    return std::nullopt;
}

MissionProcessing::RockFrame MissionProcessing::detect_rocks(
    const cv::Mat & bgr, const sensor_msgs::msg::PointCloud2 & cloud,
    const geometry_msgs::msg::TransformStamped & cloud_to_global) const
{
    const CloudReader reader(cloud);  // Reuse the existing validated XYZ reader.
    if (bgr.type() != CV_8UC3 || bgr.cols != static_cast<int>(cloud.width) ||
        bgr.rows != static_cast<int>(cloud.height)) {
        throw std::invalid_argument("Rock RGB and organized depth dimensions/types are invalid");
    }
    const auto & cfg = config_.rock;
    cv::Mat hsv, candidates;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, cfg.min_brightness),
        cv::Scalar(179, cfg.max_saturation, cfg.max_brightness), candidates);
    tf2::Transform transform;
    tf2::fromMsg(cloud_to_global.transform, transform);
    const int width = bgr.cols, height = bgr.rows;
    std::vector<geometry_msgs::msg::Point> points(bgr.total());
    const double max_range_squared = cfg.max_range * cfg.max_range;
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            auto & eligible = candidates.at<std::uint8_t>(v, u);
            if (!eligible) {
                continue;
            }
            const auto p = reader.at(u, v);
            if (!finite_point(p) || p.x <= 0.0 ||
                p.x * p.x + p.y * p.y + p.z * p.z > max_range_squared) {
                eligible = 0;
                continue;
            }
            // This project's cloud is camera_link (X forward), not optical Z forward.
            const auto q = transform * tf2::Vector3(p.x, p.y, p.z);
            geometry_msgs::msg::Point world;
            world.x = q.x();
            world.y = q.y();
            world.z = q.z();
            const double above_ground = world.z - cfg.ground_z;
            if (!finite_point(world) || above_ground < cfg.min_height || above_ground > cfg.max_height) {
                eligible = 0;
                continue;
            }
            points[static_cast<std::size_t>(v) * width + u] = world;
        }
    }
    RockFrame result;
    result.header = cloud.header;
    result.mask = cv::Mat::zeros(bgr.size(), CV_8UC1);
    // Four-connected flood fill with a 3D distance test on each edge.
    constexpr std::array<int, 4> dx{{-1, 1, 0, 0}};
    constexpr std::array<int, 4> dy{{0, 0, -1, 1}};
    const double gap_squared = cfg.neighbour_distance * cfg.neighbour_distance;
    std::vector<std::size_t> component;
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            if (!candidates.at<std::uint8_t>(v, u)) {
                continue;
            }
            candidates.at<std::uint8_t>(v, u) = 0;
            component.clear();
            component.push_back(static_cast<std::size_t>(v) * width + u);
            double min_x = points[component.front()].x, max_x = min_x;
            double min_y = points[component.front()].y, max_y = min_y;
            for (std::size_t head = 0; head < component.size(); ++head) {
                const auto index = component[head];
                const auto & point = points[index];
                min_x = std::min(min_x, point.x);
                max_x = std::max(max_x, point.x);
                min_y = std::min(min_y, point.y);
                max_y = std::max(max_y, point.y);
                const int x = static_cast<int>(index % width), y = static_cast<int>(index / width);
                for (std::size_t k = 0; k < dx.size(); ++k) {
                    const int nx = x + dx[k], ny = y + dy[k];
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height ||
                        !candidates.at<std::uint8_t>(ny, nx)) {
                        continue;
                    }
                    const auto next = static_cast<std::size_t>(ny) * width + nx;
                    if (squared_distance(point, points[next]) <= gap_squared) {
                        candidates.at<std::uint8_t>(ny, nx) = 0;
                        component.push_back(next);
                    }
                }
            }
            if (component.size() < static_cast<std::size_t>(cfg.min_component_pixels) ||
                std::max(max_x - min_x, max_y - min_y) < cfg.min_horizontal_extent) {
                continue;
            }
            for (const auto index : component) {
                result.mask.at<std::uint8_t>(static_cast<int>(index / width),
                    static_cast<int>(index % width)) = 255;
                result.points.push_back(points[index]);
            }
        }
    }
    return result;
}
