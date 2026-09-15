#include "wombat/wombat_node.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

WombatNode::WombatNode() : Node("wombat_node"),
	processing_(
		declare_parameter("initial_side_length", 2.0),
		declare_parameter("side_length_increment", 2.0),
		declare_parameter("search_corridor_width", 1.0))
{
	// Parameters keep search geometry, frame names, and inspection duration
	// adjustable from launch or YAML without rebuilding the node.
	global_frame_ = declare_parameter("global_frame", "map");
	robot_base_frame_ = declare_parameter("robot_base_frame", "base_link");
	inspection_pause_seconds_ = declare_parameter("inspection_pause_seconds", 5.0);
	const auto weed_topic = declare_parameter(
		"weed_locations_topic", "/weed");

	// PoseArray is intentionally the only perception dependency. A separate
	// plant-perception node can publish detections without being coupled here.
	weed_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
		weed_topic, rclcpp::QoS(10),
		std::bind(&WombatNode::weed_callback, this, std::placeholders::_1));
	// The action name is relative, so the launch namespace resolves it to
	// the system namespace, for example /wombat/navigate_to_pose.
	navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
		this, "navigate_to_pose");
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	// Nav2 handles continuous motion between goals, so a modest control rate is
	// sufficient for selecting the next goal.
	control_timer_ = create_wall_timer(
		500ms, std::bind(&WombatNode::control_timer_callback, this));

	RCLCPP_INFO(get_logger(), "Wombat planner listening on %s", weed_topic.c_str());
}

void WombatNode::weed_callback(
	const geometry_msgs::msg::PoseArray::SharedPtr message)
{
	// Explicitly framed detections must already be in map coordinates. Frame
	// conversion is deliberately outside this node for the current integration.
	if (!message->header.frame_id.empty() && message->header.frame_id != global_frame_) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Ignoring weed locations in frame '%s'; expected '%s'",
			message->header.frame_id.c_str(), global_frame_.c_str());
		return;
	}
	processing_.update_weeds(message->poses);
}

void WombatNode::control_timer_callback()
{
	// Do not submit competing goals while Nav2 is moving or the robot is paused
	// at a weed for inspection.
	if (goal_active_ || inspecting_weed_) {
		return;
	}

	geometry_msgs::msg::PoseStamped pose;
	if (!current_pose(pose)) {
		return;
	}
	// The first successful TF lookup establishes the search centre.
	processing_.set_origin(pose);

	if (!navigation_client_->wait_for_action_server(0s)) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Waiting for Nav2 navigate_to_pose action server");
		return;
	}

	// Processing decides whether this goal is a search waypoint or nearby weed.
	send_goal(processing_.choose_goal(pose));
}

bool WombatNode::current_pose(geometry_msgs::msg::PoseStamped & pose)
{
	try {
		// Nav2 and SLAM publish TF asynchronously, so startup may briefly lack
		// the transform needed to plan a goal.
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
			"Waiting for %s -> %s transform: %s", global_frame_.c_str(),
			robot_base_frame_.c_str(), error.what());
		return false;
	}
}

void WombatNode::send_goal(const WombatProcessing::Goal & goal)
{
	// Store the selected goal because the result callback must distinguish a
	// completed weed inspection from a completed search segment.
	active_goal_ = goal;
	active_goal_.pose.header.frame_id = global_frame_;
	active_goal_.pose.header.stamp = now();
	goal_active_ = true;

	NavigateToPose::Goal nav_goal;
	nav_goal.pose = active_goal_.pose;
	auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
	options.goal_response_callback =
		std::bind(&WombatNode::goal_response_callback, this, std::placeholders::_1);
	options.result_callback =
		std::bind(&WombatNode::result_callback, this, std::placeholders::_1);
	navigation_client_->async_send_goal(nav_goal, options);
}

void WombatNode::goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
{
	// A rejected goal releases the planner so the timer can retry.
	if (!goal_handle) {
		goal_active_ = false;
		RCLCPP_WARN(get_logger(), "Nav2 rejected the Wombat goal");
	}
}

void WombatNode::result_callback(const GoalHandle::WrappedResult & result)
{
	// Failed or cancelled goals remain pending; only successful navigation is
	// committed to the planner state below.
	goal_active_ = false;
	if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_WARN(get_logger(), "Nav2 did not complete the Wombat goal");
		return;
	}

	if (active_goal_.weed_index.has_value()) {
		// Reaching a weed is not enough: pause briefly so the robot can inspect it.
		inspecting_weed_ = true;
		inspection_timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::duration<double>(inspection_pause_seconds_)),
			std::bind(&WombatNode::finish_inspection, this));
	} else {
		// A completed search waypoint advances the expanding-square generator.
		processing_.complete_goal(active_goal_);
	}
}

void WombatNode::finish_inspection()
{
	// Cancel the one-shot timer before updating planner state so it cannot fire
	// repeatedly and remove the same weed more than once.
	if (inspection_timer_) {
		inspection_timer_->cancel();
	}
	processing_.complete_goal(active_goal_);
	inspecting_weed_ = false;
}