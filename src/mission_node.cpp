#include "mission_node.h"

#include <algorithm>
#include <cmath>
#include <limits>

MissionNode::MissionNode()
: Node("mission_node")
{
	global_frame_ = declare_parameter("global_frame", "map");
	wombat_base_frame_ = declare_parameter("wombat_base_frame", "base_link");
	camera_width_of_view_ = declare_parameter("camera_width_of_view", 1.0);
	lane_overlap_ = declare_parameter("lane_overlap", 0.5);
	search_goal_distance_ = declare_parameter("search_goal_distance", 1.0);

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
	initial_search_point_reached_ = false;
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
	if (region_vertices_.size() < 4) {
		search_lines_.clear();
		return;
	}

	search_lines_.clear();
	search_goal_index_ = 0;
	initial_search_point_reached_ = false;
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

	search_line_assignment_alternation = false;

	for (double y = min_y; y <= max_y + 1e-6; y += line_spacing) {
		geometry_msgs::msg::PoseStamped start = line_start;
		start.pose.position.x = min_x;
		start.pose.position.y = y;
		geometry_msgs::msg::PoseStamped end = line_start;
		end.pose.position.x = max_x;
		end.pose.position.y = y;

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

bool MissionNode::serve_next_goal(
	const std::shared_ptr<rs1::srv::WombatNextGoal::Request> request,
	std::shared_ptr<rs1::srv::WombatNextGoal::Response> response)
{
	(void)request;

	response->is_weed = false;
	response->goal.header.frame_id = global_frame_;
	response->goal.pose.orientation.w = 1.0;

	geometry_msgs::msg::PoseStamped robot_pose;
	if (!current_pose(robot_pose)) {
		response->goal.pose.position.x = 0.0;
		response->goal.pose.position.y = 0.0;
		return true;
	}

	auto line_start = search_lines_[search_goal_index_].pose.position;
	auto line_end = search_lines_[search_goal_index_ + 1].pose.position;
	double line_dx = line_end.x - line_start.x;
	double line_dy = line_end.y - line_start.y;
	double line_length = std::hypot(line_dx, line_dy);
	if (line_length == 0.0) {
		response->goal.pose.position = line_end;
		return true;
	}

	// Before starting the search, drive all the way to the first point of the
	// first lane. Weeds and lookahead are ignored during this initial approach.
	if (!initial_search_point_reached_) {
		const double distance_to_start = std::hypot(
			robot_pose.pose.position.x - line_start.x,
			robot_pose.pose.position.y - line_start.y);
		if (distance_to_start > 0.25) {
			response->goal.pose.position = line_start;
			return true;
		}
		initial_search_point_reached_ = true;
	}

	line_dx /= line_length;
	line_dy /= line_length;
	double robot_progress = std::clamp(
		(robot_pose.pose.position.x - line_start.x) * line_dx +
		(robot_pose.pose.position.y - line_start.y) * line_dy,
		0.0, line_length);
	if (robot_progress >= line_length - 0.05) {
		search_goal_index_ += 2;
		if (search_goal_index_ + 1 >= search_lines_.size()) {
			search_goal_index_ = 0;
		}
		line_start = search_lines_[search_goal_index_].pose.position;
		line_end = search_lines_[search_goal_index_ + 1].pose.position;
		line_dx = line_end.x - line_start.x;
		line_dy = line_end.y - line_start.y;
		line_length = std::hypot(line_dx, line_dy);
		if (line_length == 0.0) {
			response->goal.pose.position = line_end;
			return true;
		}
		line_dx /= line_length;
		line_dy /= line_length;
		robot_progress = std::clamp(
			(robot_pose.pose.position.x - line_start.x) * line_dx +
			(robot_pose.pose.position.y - line_start.y) * line_dy,
			0.0, line_length);
	}
	const double lane_half_width = camera_width_of_view_ * 0.25;
	const double lookahead = std::max(0.0, search_goal_distance_);

	// Choose the closest weed ahead on this lane, provided it is inside half
	// the distance between adjacent search lines and inside the lookahead.
	std::size_t selected_weed = weeds_.size();
	double selected_progress = std::numeric_limits<double>::max();
	for (std::size_t index = 0; index < weeds_.size(); ++index) {
		const auto & weed = weeds_[index].pose.position;
		const double weed_progress =
			(weed.x - line_start.x) * line_dx + (weed.y - line_start.y) * line_dy;
		const double lateral_distance = std::abs(
			(weed.x - line_start.x) * line_dy - (weed.y - line_start.y) * line_dx);
		if (weed_progress > robot_progress &&
			weed_progress <= std::min(line_length, robot_progress + lookahead) &&
			lateral_distance <= lane_half_width &&
			weed_progress < selected_progress) {
			selected_weed = index;
			selected_progress = weed_progress;
		}
	}

	if (selected_weed < weeds_.size()) {
		response->goal.pose.position = weeds_[selected_weed].pose.position;
		response->goal.pose.orientation.w = 1.0;
		response->is_weed = true;
		return true;
	}

	// No weed is ready for inspection, so return a point lookahead metres
	// along the current lane, stopping at the lane endpoint.
	const double goal_progress = std::min(line_length, robot_progress + lookahead);
	response->goal.pose.position.x = line_start.x + goal_progress * line_dx;
	response->goal.pose.position.y = line_start.y + goal_progress * line_dy;
	const double heading = std::atan2(line_dy, line_dx);
	response->goal.pose.orientation.z = std::sin(heading / 2.0);
	response->goal.pose.orientation.w = std::cos(heading / 2.0);

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