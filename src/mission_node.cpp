#include "mission_node.h"

#include <algorithm>
#include <cmath>
#include <limits>

MissionNode::MissionNode()
: Node("mission_node")
{
	global_frame_ = declare_parameter("global_frame", "map");
	robot_base_frame_ = declare_parameter("robot_base_frame", "base_link");
	camera_width_of_view_ = declare_parameter("camera_width_of_view", 1.0);
	lane_overlap_ = declare_parameter("lane_overlap", 0.5);

	region_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
		"/region", rclcpp::QoS(10),
		std::bind(&MissionNode::region_callback, this, std::placeholders::_1));
	weed_subscription_ = create_subscription<rs1::msg::PlantDetectionArray>(
		"/plant_detection/weeds", rclcpp::QoS(10),
		std::bind(&MissionNode::weed_callback, this, std::placeholders::_1));
	next_goal_service_ = create_service<rs1::srv::WombatNextGoal>(
		"wombat_next_goal",
		std::bind(&MissionNode::serve_next_goal, this, std::placeholders::_1, std::placeholders::_2));

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	RCLCPP_INFO(get_logger(), "Mission service ready; waiting for region and weed data");
}

void MissionNode::region_callback(const geometry_msgs::msg::PoseArray::SharedPtr message)
{
	if (message->poses.empty()) {
		return;
	}
	region_vertices_ = message->poses;
	build_search_lines();
	RCLCPP_INFO(get_logger(), "Region updated with %zu vertices", region_vertices_.size());
}

void MissionNode::weed_callback(const rs1::msg::PlantDetectionArray::SharedPtr message)
{
	weeds_ = message->detections;
}

bool MissionNode::current_pose(geometry_msgs::msg::PoseStamped & pose)
{
	try {
		const auto transform = tf_buffer_->lookupTransform(
			global_frame_, robot_base_frame_, tf2::TimePointZero);
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
			global_frame_.c_str(), robot_base_frame_.c_str(), error.what());
		return false;
	}
}

void MissionNode::build_search_lines()
{
	if (region_vertices_.size() < 4) {
		search_lines_.clear();
		return;
	}

	search_lines_.clear();
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
	const double line_spacing = std::max(camera_width_of_view_ * 0.5, 0.5);

	geometry_msgs::msg::PoseStamped line_start;
	line_start.header.frame_id = global_frame_;
	line_start.pose.position.z = 0.0;
	line_start.pose.orientation.w = 1.0;

	for (double y = min_y; y <= max_y + 1e-6; y += line_spacing) {
		geometry_msgs::msg::PoseStamped start = line_start;
		start.pose.position.x = min_x;
		start.pose.position.y = y;
		geometry_msgs::msg::PoseStamped end = line_start;
		end.pose.position.x = max_x;
		end.pose.position.y = y;
		search_lines_.push_back(start);
		search_lines_.push_back(end);
	}
}

geometry_msgs::msg::PoseStamped MissionNode::next_search_goal()
{
	geometry_msgs::msg::PoseStamped result;
	result.header.frame_id = global_frame_;
	result.pose.orientation.w = 1.0;

	if (search_lines_.empty()) {
		result.pose.position.x = 0.0;
		result.pose.position.y = 0.0;
		return result;
	}

	if (lane_index_ >= search_lines_.size()) {
		lane_index_ = 0;
		forward_direction_ = !forward_direction_;
	}

	result = search_lines_[lane_index_];
	if (forward_direction_) {
		lane_index_ += 1;
	} else {
		lane_index_ += 1;
	}
	return result;
}

bool MissionNode::serve_next_goal(
	const std::shared_ptr<rs1::srv::WombatNextGoal::Request> request,
	std::shared_ptr<rs1::srv::WombatNextGoal::Response> response)
{
	(void)request;

	geometry_msgs::msg::PoseStamped robot_pose;
	if (!current_pose(robot_pose)) {
		response->goal.header.frame_id = global_frame_;
		response->goal.pose.position.x = 0.0;
		response->goal.pose.position.y = 0.0;
		response->goal.pose.orientation.w = 1.0;
		response->is_weed = false;
		return true;
	}

	geometry_msgs::msg::PoseStamped search_goal = next_search_goal();
	search_goal.pose.position.x += 0.25;
	search_goal.pose.position.y += 0.0;

	response->is_weed = false;
	response->plant_type = "";
	response->goal = search_goal;

	for (const auto & weed : weeds_) {
		const double dx = weed.pose.position.x - robot_pose.pose.position.x;
		const double dy = weed.pose.position.y - robot_pose.pose.position.y;
		if (std::hypot(dx, dy) <= camera_width_of_view_ * 0.5) {
			response->goal.pose.position = weed.pose.position;
			response->goal.pose.orientation.w = 1.0;
			response->is_weed = true;
			response->plant_type = weed.plant_type;
			break;
		}
	}

	return true;
}

double MissionNode::distance(
	const geometry_msgs::msg::Point & first,
	const geometry_msgs::msg::Point & second)
{
	return std::hypot(first.x - second.x, first.y - second.y);
}

double MissionNode::point_to_segment_distance(
	const geometry_msgs::msg::Point & point,
	const geometry_msgs::msg::Point & segment_start,
	const geometry_msgs::msg::Point & segment_end)
{
	const double dx = segment_end.x - segment_start.x;
	const double dy = segment_end.y - segment_start.y;
	const double length_squared = dx * dx + dy * dy;
	if (length_squared == 0.0) {
		return distance(point, segment_start);
	}
	const double projection = std::clamp(
		((point.x - segment_start.x) * dx + (point.y - segment_start.y) * dy) / length_squared,
		0.0,
		1.0);
	geometry_msgs::msg::Point nearest = segment_start;
	nearest.x += projection * dx;
	nearest.y += projection * dy;
	return distance(point, nearest);
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MissionNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}