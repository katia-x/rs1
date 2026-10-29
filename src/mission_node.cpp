#include "mission_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

// Configure mission parameters, subscriptions, the goal service, and TF.
MissionNode::MissionNode()
: Node("mission_node")
{
	global_frame_ = declare_parameter("global_frame", "map");
	wombat_base_frame_ = declare_parameter("wombat_base_frame", "husky1_base_link");
	const double camera_width_of_view = declare_parameter("camera_width_of_view", 2.5);
	search_lane_spacing_ = camera_width_of_view * 0.5;
	search_goal_distance_ = declare_parameter("search_goal_distance", search_goal_distance_);
	location_tolerance_ = declare_parameter("location_tolerance", location_tolerance_);
	if (!std::isfinite(search_lane_spacing_) || search_lane_spacing_ <= 0.0) {
		throw std::invalid_argument("camera_width_of_view must be positive and finite");
	}
	const auto detections_topic = declare_parameter<std::string>(
		"detections_topic", "/husky1/detections");
	const auto depth_topic = declare_parameter<std::string>(
		"depth_topic", "/husky1/camera/depth/points");

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
	MissionProcessing::Config weed_config;
	weed_config.global_frame = global_frame_;
	// These settings configure the model at construction; do not silently accept
	// runtime parameter changes that the model would not apply.
	const auto weed_parameter = [this](const std::string & name, const auto & value) {
		rcl_interfaces::msg::ParameterDescriptor descriptor;
		descriptor.read_only = true;
		return declare_parameter("weed." + name, value, descriptor);
	};
	weed_config.confidence_threshold =
		weed_parameter("confidence_threshold", weed_config.confidence_threshold);
	weed_config.max_time_difference =
		weed_parameter("max_time_difference", weed_config.max_time_difference);
	weed_config.max_observation_age =
		weed_parameter("max_observation_age", weed_config.max_observation_age);
	weed_config.roi_fraction =
		weed_parameter("roi_fraction", weed_config.roi_fraction);
	weed_config.depth_cluster_tolerance =
		weed_parameter("depth_cluster_tolerance", weed_config.depth_cluster_tolerance);
	weed_config.min_valid_samples =
		weed_parameter("min_valid_samples", weed_config.min_valid_samples);
	weed_config.min_cluster_fraction =
		weed_parameter("min_cluster_fraction", weed_config.min_cluster_fraction);
	weed_config.association_distance =
		weed_parameter("association_distance", weed_config.association_distance);
	weed_config.smoothing_factor =
		weed_parameter("smoothing_factor", weed_config.smoothing_factor);
	weed_config.track_timeout =
		weed_parameter("track_timeout", weed_config.track_timeout);
	mission_processing_ = std::make_unique<MissionProcessing>(weed_config, tf_buffer_);

	region_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
		"/region", rclcpp::QoS(10),
		std::bind(&MissionNode::region_callback, this, std::placeholders::_1));
	detection_subscription_ = create_subscription<vision_msgs::msg::Detection2DArray>(
		detections_topic, rclcpp::SensorDataQoS(),
		[this](vision_msgs::msg::Detection2DArray::ConstSharedPtr detections) {
			mission_processing_->receive_detections(std::move(detections));
		});
	depth_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
		// The Gazebo bridge offers reliable delivery. Keep enough queued frames
		// for detection/depth matching instead of requesting best-effort clouds.
		depth_topic, rclcpp::QoS(10).reliable(),
		[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
			mission_processing_->receive_cloud(std::move(cloud));
		});
	target_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("/husky1/target_weed", 5);
	target_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
		"/husky1/target_weed_marker", rclcpp::QoS(1).transient_local());
	estimate_markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
		"/husky1/weed_estimate_markers", rclcpp::QoS(1).transient_local());
	permanent_weed_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
		"/husky1/permanent_weed_exclusions", rclcpp::QoS(1).transient_local());
	weed_timer_ = create_wall_timer(std::chrono::milliseconds(200),
		std::bind(&MissionNode::update_weed_target, this));
	next_goal_service_ = create_service<rs1::srv::WombatNextGoal>(
		"wombat_next_goal",
		std::bind(&MissionNode::serve_next_goal, this, std::placeholders::_1, std::placeholders::_2));

	RCLCPP_INFO(get_logger(), "Mission service ready; waiting for region and weed data");
}

// Replace the current search region and rebuild its waypoint pattern.
void MissionNode::region_callback(const geometry_msgs::msg::PoseArray::SharedPtr message)
{
	if (message->poses.size() < 3 || std::any_of(message->poses.begin(), message->poses.end(),
		[](const auto & pose) {
			return !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(pose.position.z);
		})) {
		RCLCPP_WARN(get_logger(), "Ignoring region without at least three finite vertices");
		return;
	}
	if (message->header.frame_id != global_frame_) {
		RCLCPP_WARN(
			get_logger(),
			"Ignoring region in frame '%s'; expected '%s'",
			message->header.frame_id.c_str(), global_frame_.c_str());
		return;
	}
	const bool same_region = region_vertices_.size() == message->poses.size() &&
		std::equal(region_vertices_.begin(), region_vertices_.end(), message->poses.begin(),
			[](const auto & current, const auto & incoming) {
				return current.position.x == incoming.position.x &&
					current.position.y == incoming.position.y &&
					current.position.z == incoming.position.z;
			});
	if (same_region) {
		RCLCPP_DEBUG(get_logger(), "Ignoring duplicate region snapshot");
		return;
	}
	region_vertices_ = message->poses;
	build_search_lines();
	clear_weed_target();
	RCLCPP_INFO(get_logger(), "Region updated with %zu vertices", region_vertices_.size());
}

void MissionNode::clear_weed_target()
{
	selected_weed_.reset();
	selected_weed_id_.reset();
	if (!target_visible_) { return; }
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = global_frame_;
	marker.header.stamp = now();
	marker.ns = "target_weed";
	marker.id = 0;
	marker.action = visualization_msgs::msg::Marker::DELETE;
	target_marker_publisher_->publish(marker);
	target_visible_ = false;
}

void MissionNode::update_weed_target()
{
	const auto time = now();
	const auto result = mission_processing_->update(time);
	if (result == MissionProcessing::UpdateResult::WaitingForTransform) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
			"Waiting for acquisition-time depth-to-map TF for weed estimation");
	} else if (result == MissionProcessing::UpdateResult::WaitingForCloud) {
		const auto timing = mission_processing_->cloud_timing();
		if (!timing.nearest_cloud_stamp) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
				"Weed estimation: no depth clouds received on %s", depth_subscription_->get_topic_name());
		} else {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
				"Weed depth match: detection=%.6f s, nearest_cloud=%.6f s, delta=%+.6f s, "
				"limit=%.3f s, buffered=%zu, pending=%zu, now=%.6f s",
				timing.detection_stamp->seconds(), timing.nearest_cloud_stamp->seconds(),
				(*timing.nearest_cloud_stamp - *timing.detection_stamp).seconds(),
				mission_processing_->config().max_time_difference, timing.buffered_clouds,
				timing.pending_batches, time.seconds());
		}
	} else if (result == MissionProcessing::UpdateResult::Rejected) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
			"Rejected stale or incompatible weed/depth observation");
	}
	// Perception remains visible even without an eligible current-lane target.
	publish_weed_estimates(time);
	geometry_msgs::msg::PoseArray permanent_weeds;
	permanent_weeds.header.frame_id = global_frame_;
	permanent_weeds.header.stamp = time;
	for (const auto & weed : mission_processing_->permanent_weeds()) {
		geometry_msgs::msg::Pose pose;
		pose.position = weed.position;
		pose.orientation.w = 1.0;
		permanent_weeds.poses.push_back(pose);
	}
	permanent_weed_publisher_->publish(permanent_weeds);
	// Odd endpoints are active search lanes. Even endpoints are connectors
	// between lanes; no weed is selected during those transitions.
	if (search_goal_index_ == 0 || search_goal_index_ >= search_lines_.size() || search_goal_index_ % 2 == 0) {
		clear_weed_target(); return;
	}
	geometry_msgs::msg::PoseStamped robot;
	if (!current_pose(robot)) { clear_weed_target(); return; }
	const MissionProcessing::SearchLane lane{
		search_lines_[search_goal_index_ - 1].pose.position,
		search_lines_[search_goal_index_].pose.position,
		search_lane_spacing_ * 0.5};
	const auto target = mission_processing_->select_target(
		robot.pose.position, lane, region_vertices_, selected_weed_id_);
	if (!target) { clear_weed_target(); return; }
	selected_weed_ = target;
	selected_weed_id_ = target->id;
	geometry_msgs::msg::PointStamped point;
	point.header.frame_id = global_frame_;
	point.header.stamp = target->last_seen;
	point.point = target->position;
	target_publisher_->publish(point);
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = global_frame_;
	marker.header.stamp = time;
	marker.ns = "target_weed";
	marker.id = 0;  // Replaces the previous target rather than leaving ghost markers.
	marker.type = visualization_msgs::msg::Marker::SPHERE;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.position = target->position;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
	marker.color.r = 1.0f; marker.color.g = 0.8f; marker.color.a = 1.0f;
	marker.lifetime = rclcpp::Duration::from_seconds(std::max(0.01,
		mission_processing_->config().track_timeout - (time - target->last_seen).seconds()));
	target_marker_publisher_->publish(marker);
	target_visible_ = true;
}

void MissionNode::publish_weed_estimates(const rclcpp::Time & time)
{
	visualization_msgs::msg::MarkerArray snapshot;
	const auto & estimates = mission_processing_->estimates();
	snapshot.markers.reserve(estimates.size() + 1);
	// Each message is a complete snapshot on this dedicated topic. Clearing first
	// removes expired tracks and lets late RViz subscribers reconstruct the display.
	visualization_msgs::msg::Marker clear;
	clear.header.frame_id = global_frame_;
	clear.header.stamp = time;
	clear.action = visualization_msgs::msg::Marker::DELETEALL;
	snapshot.markers.push_back(clear);
	for (const auto & estimate : estimates) {
		visualization_msgs::msg::Marker marker;
		marker.header.frame_id = global_frame_;
		marker.header.stamp = time;
		marker.ns = "weed_estimates";
		marker.id = static_cast<std::int32_t>(snapshot.markers.size() - 1);
		marker.type = visualization_msgs::msg::Marker::SPHERE;
		marker.action = visualization_msgs::msg::Marker::ADD;
		marker.pose.position = estimate.position;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = marker.scale.y = marker.scale.z = 0.12;
		marker.color.g = 0.7f; marker.color.b = 1.0f; marker.color.a = 1.0f;
		marker.lifetime = rclcpp::Duration::from_seconds(std::max(0.01,
			mission_processing_->config().track_timeout - (time - estimate.last_seen).seconds()));
		snapshot.markers.push_back(std::move(marker));
	}
	estimate_markers_publisher_->publish(snapshot);
}

// Look up Wombat's current pose in the global planning frame.
bool MissionNode::current_pose(geometry_msgs::msg::PoseStamped & pose)
{
	try {
		const auto transform = tf_buffer_->lookupTransform(
			global_frame_, wombat_base_frame_, tf2::TimePointZero);
		pose.header = transform.header;
		pose.pose.position.x = transform.transform.translation.x;
		pose.pose.position.y = transform.transform.translation.y;
		pose.pose.position.z = transform.transform.translation.z;
		pose.pose.orientation = transform.transform.rotation;
		return true;
	} catch (const tf2::TransformException & error) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Waiting for %s -> %s transform: %s",
			global_frame_.c_str(), wombat_base_frame_.c_str(), error.what());
		return false;
	}
}

void MissionNode::build_search_lines()
{
	search_lines_.clear();
	search_goal_index_ = 0;

	std::vector<double> xs;
	std::vector<double> ys;
	for (const auto & vertex : region_vertices_) {
		xs.push_back(vertex.position.x);
		ys.push_back(vertex.position.y);
	}

	const double min_x = *std::min_element(xs.begin(), xs.end());
	const double max_x = *std::max_element(xs.begin(), xs.end());
	const double min_y = *std::min_element(ys.begin(), ys.end());
	const double max_y = *std::max_element(ys.begin(), ys.end());

	geometry_msgs::msg::PoseStamped line_start;
	line_start.header.frame_id = global_frame_;
	line_start.pose.position.z = 0.0;
	line_start.pose.orientation.w = 1.0;

	search_line_assignment_alternation = false; //initial state (not a configuration option)

	for (double x = min_x; x <= max_x + 1e-6; x += search_lane_spacing_) {
		geometry_msgs::msg::PoseStamped start = line_start;
		start.pose.position.y = min_y;
		start.pose.position.x = x;
		geometry_msgs::msg::PoseStamped end = line_start;
		end.pose.position.y = max_y;
		end.pose.position.x = x;

		if (search_line_assignment_alternation){
			search_lines_.push_back(end);
			search_lines_.push_back(start);
			search_line_assignment_alternation = false;
		} else {
			search_lines_.push_back(start);
			search_lines_.push_back(end);
			search_line_assignment_alternation = true;
		}
	}
}

// Continue returning search goals, or a nearby selected weed target.
void MissionNode::serve_next_goal(
	const std::shared_ptr<rs1::srv::WombatNextGoal::Request> request,
	std::shared_ptr<rs1::srv::WombatNextGoal::Response> response)
{
	response->is_weed = false;
	response->success = true;
	response->goal.header.frame_id = global_frame_;
	response->goal.pose.orientation.w = 1.0;
	response->weed_id = 0; //null

	geometry_msgs::msg::PoseStamped robot_pose;
	if (!current_pose(robot_pose)) {
		response->success = false;
		response->goal.pose.position.x = 0.0;
		response->goal.pose.position.y = 0.0;
		return;
	}
	if (search_lines_.empty()) {
		response->success = false;
		return;
	}

	if (request->skip_current_goal) {
		++consecutive_goal_failures_;
		RCLCPP_WARN(get_logger(),
			"Skipping failed waypoint (failure %zu) and extending the next goal along the current lane",
			consecutive_goal_failures_);
	} else {
		consecutive_goal_failures_ = 0;
	}
	const double lookahead = std::max(0.0, search_goal_distance_);
	const bool skipping_lane_endpoint = request->skip_current_goal && last_goal_valid_ &&
		!last_goal_was_weed_ && search_goal_index_ != 0 &&
		search_goal_index_ % 2 == 1 &&
		distance(last_goal_position_, search_lines_[search_goal_index_].pose.position) < location_tolerance_;
	const bool extend_failed_goal = request->skip_current_goal && last_goal_valid_ &&
		!skipping_lane_endpoint;
	if (skipping_lane_endpoint) {
		search_goal_index_++;
		if (search_goal_index_ >= search_lines_.size()) {
			search_goal_index_ = 0;
		}
		RCLCPP_WARN(get_logger(), "Skipped failed lane endpoint and advanced to the next search segment");
	}

	// Move to the next lane before generating another goal once the current
	// lane endpoint has been reached.
	if (search_goal_index_ != 0 && distance(search_lines_[search_goal_index_].pose.position, robot_pose.pose.position) < location_tolerance_) {
		search_goal_index_++;
		if (search_goal_index_ >= search_lines_.size()) {
			search_goal_index_ = 0;
		}
	}

	// Before starting the search, drive all the way to the first point of the
	// first lane. Weeds and lookahead are ignored during this initial approach.
	if (search_goal_index_ == 0) {
		response->goal.pose.position.x = search_lines_[search_goal_index_].pose.position.x;
		response->goal.pose.position.y = search_lines_[search_goal_index_].pose.position.y;
		const auto & lane_start = search_lines_[0].pose.position;
		const auto & lane_end = search_lines_[1].pose.position;
		const double heading = std::atan2(
			lane_end.y - lane_start.y, lane_end.x - lane_start.x);
		response->goal.pose.orientation.z = std::sin(heading * 0.5);
		response->goal.pose.orientation.w = std::cos(heading * 0.5);
		last_goal_position_ = response->goal.pose.position;
		last_goal_valid_ = true;
		last_goal_was_weed_ = false;
		search_goal_index_++;
		return;
	}

	if (!request->skip_current_goal && selected_weed_ &&
		distance(robot_pose.pose.position, selected_weed_->position) < search_goal_distance_) {
		response->goal.pose.position = selected_weed_->position;
		response->is_weed = true;
		response->weed_id = selected_weed_->id;
	} else {
		response->goal.pose.position.x = search_lines_[search_goal_index_].pose.position.x;
		if (extend_failed_goal) {
			const auto & lane_start = search_lines_[search_goal_index_ - 1].pose.position;
			const auto & lane_end = search_lines_[search_goal_index_].pose.position;
			const double lane_length = distance(lane_start, lane_end);
			if (lane_length > 1e-6) {
				const double extra = 0.5 * lookahead *
					static_cast<double>(consecutive_goal_failures_);
				response->goal.pose.position.x = last_goal_position_.x +
					(extra / lane_length) * (lane_end.x - lane_start.x);
				response->goal.pose.position.y = last_goal_position_.y +
					(extra / lane_length) * (lane_end.y - lane_start.y);
			}
		} else {
			const double goal_lookahead = lookahead * (1.0 +
				0.5 * static_cast<double>(consecutive_goal_failures_));
			if (robot_pose.pose.position.y < search_lines_[search_goal_index_].pose.position.y) {
				response->goal.pose.position.y = std::min(
					robot_pose.pose.position.y + goal_lookahead,
					search_lines_[search_goal_index_].pose.position.y);
			} else {
				response->goal.pose.position.y = std::max(
					robot_pose.pose.position.y - goal_lookahead,
					search_lines_[search_goal_index_].pose.position.y);
			}
		}
	}
	const auto & lane_start = search_lines_[search_goal_index_ - 1].pose.position;
	const auto & lane_end = search_lines_[search_goal_index_].pose.position;
	const double heading = std::atan2(
		lane_end.y - lane_start.y, lane_end.x - lane_start.x);
	response->goal.pose.orientation.z = std::sin(heading * 0.5);
	response->goal.pose.orientation.w = std::cos(heading * 0.5);
	last_goal_position_ = response->goal.pose.position;
	last_goal_valid_ = true;
	last_goal_was_weed_ = response->is_weed;
	return;
}

// Return the planar distance between two points.
double MissionNode::distance(
	const geometry_msgs::msg::Point & first,
	const geometry_msgs::msg::Point & second)
{
	return std::hypot(first.x - second.x, first.y - second.y);
}
