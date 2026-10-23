#include "mission_node.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>

// Configure mission parameters, subscriptions, the goal service, and TF.
MissionNode::MissionNode()
: Node("mission_node")
{
	global_frame_ = declare_parameter("global_frame", "map");
	wombat_base_frame_ = declare_parameter("wombat_base_frame", "husky1_base_link");
	camera_width_of_view_ = declare_parameter("camera_width_of_view", camera_width_of_view_);
	search_goal_distance_ = declare_parameter("search_goal_distance", search_goal_distance_);
	location_tolerance_ = declare_parameter("location_tolerance", location_tolerance_);
	const auto detections_topic = declare_parameter<std::string>(
		"detections_topic", "/husky1/detections");

	region_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
		"/region", rclcpp::QoS(10),
		std::bind(&MissionNode::region_callback, this, std::placeholders::_1));
	detection_subscription_ = create_subscription<vision_msgs::msg::Detection2DArray>(
		detections_topic, rclcpp::SensorDataQoS(),
		std::bind(&MissionNode::detection_callback, this, std::placeholders::_1));
	next_goal_service_ = create_service<rs1::srv::WombatNextGoal>(
		"wombat_next_goal",
		std::bind(&MissionNode::serve_next_goal, this, std::placeholders::_1, std::placeholders::_2));

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	RCLCPP_INFO(get_logger(), "Mission service ready; waiting for region and weed data");
}

// Replace the current search region and rebuild its waypoint pattern.
void MissionNode::region_callback(const geometry_msgs::msg::PoseArray::SharedPtr message)
{
	if (message->poses.empty()) {
		return;
	}
	if (message->header.frame_id != global_frame_) {
		RCLCPP_WARN(
			get_logger(),
			"Ignoring region in frame '%s'; expected '%s'",
			message->header.frame_id.c_str(), global_frame_.c_str());
		return;
	}
	region_vertices_ = message->poses;
	build_search_lines();
	RCLCPP_INFO(get_logger(), "Region updated with %zu vertices", region_vertices_.size());
}

// Store the latest image-space detections. They require depth and TF projection
// before they can safely be converted into map-frame navigation goals.
void MissionNode::detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr message)
{
	detections_ = message->detections;
}

// Look up Wombat's current pose in the global planning frame.
bool MissionNode::current_pose(geometry_msgs::msg::PoseStamped & pose)
{
	try {
		const auto transform = tf_buffer_->lookupTransform(
// Build alternating vertical search lanes across the region's bounding box.
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
	const double line_spacing = camera_width_of_view_ * 0.5;

	geometry_msgs::msg::PoseStamped line_start;
	line_start.header.frame_id = global_frame_;
	line_start.pose.position.z = 0.0;
	line_start.pose.orientation.w = 1.0;

	search_line_assignment_alternation = false; //initial state (not a configuration option)

	for (double x = min_x; x <= max_x + 1e-6; x += line_spacing) {
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

// Compute and return the next short-range goal: first approach the initial
// lane point, then prioritize a nearby weed or continue along the active lane.
void MissionNode::serve_next_goal(
	const std::shared_ptr<rs1::srv::WombatNextGoal::Request> request,
	std::shared_ptr<rs1::srv::WombatNextGoal::Response> response)
{
	response->is_weed = false;
	response->success = true;
	response->goal.header.frame_id = global_frame_;
	response->goal.pose.orientation.w = 1.0;

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

	if (request->skip_current_goal && search_goal_index_ != 0) {
		search_goal_index_++;
		if (search_goal_index_ >= search_lines_.size()) {
			search_goal_index_ = 0;
		}
		RCLCPP_WARN(get_logger(), "Skipping failed search goal and advancing to the next search segment");
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
		search_goal_index_++;
		return;
	}

	const double lookahead = std::max(0.0, search_goal_distance_);
	response->goal.pose.position.x = search_lines_[search_goal_index_].pose.position.x;
	if (robot_pose.pose.position.y < search_lines_[search_goal_index_].pose.position.y) {
		response->goal.pose.position.y = std::min(
			robot_pose.pose.position.y + lookahead,
			search_lines_[search_goal_index_].pose.position.y);
	} else {
		response->goal.pose.position.y = std::max(
			robot_pose.pose.position.y - lookahead,
			search_lines_[search_goal_index_].pose.position.y);
	}

	const double heading = std::atan2(
		response->goal.pose.position.y - robot_pose.pose.position.y,
		response->goal.pose.position.x - robot_pose.pose.position.x);
	response->goal.pose.orientation.z = std::sin(heading * 0.5);
	response->goal.pose.orientation.w = std::cos(heading * 0.5);
	return;
}

// Return the planar distance between two points.
double MissionNode::distance(
	const geometry_msgs::msg::Point & first,
	const geometry_msgs::msg::Point & second)
{
	return std::hypot(first.x - second.x, first.y - second.y);
}

// Start ROS, construct the mission node, process callbacks, and shut down.
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MissionNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}