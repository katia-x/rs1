#include "wombat/wombat_processing.h"

#include <algorithm>
#include <cmath>
#include <limits>

WombatProcessing::WombatProcessing(	double initial_side_length, double side_length_increment, double search_corridor_width)
:   initial_side_length_(initial_side_length),
	side_length_increment_(side_length_increment),
	search_corridor_width_(search_corridor_width)
{}

void WombatProcessing::set_origin(
	const geometry_msgs::msg::PoseStamped & origin)
{
	// Capture the initial map pose once. The expanding square remains anchored
	// here rather than being recalculated around the moving robot.
	if (!has_origin_) {
		origin_ = origin;
		has_origin_ = true;
	}
}

bool WombatProcessing::has_origin() const
{
	return has_origin_;
}

void WombatProcessing::update_weeds(const std::vector<geometry_msgs::msg::Pose> & weeds)
{
	// PoseArray is treated as the latest detection snapshot. Duplicate handling
	// is intentionally left to another system, as agreed for this integration.
	weeds_ = weeds;
}

WombatProcessing::Goal WombatProcessing::choose_goal(const geometry_msgs::msg::PoseStamped & current_pose)
{
	// Start with the next search waypoint. A nearby weed replaces this default
	// only when it lies on the upcoming side of the expanding square.
	Goal result{next_search_waypoint(), std::nullopt};
	const double dx = result.pose.pose.position.x - current_pose.pose.position.x;
	const double dy = result.pose.pose.position.y - current_pose.pose.position.y;
	const double segment_length_squared = dx * dx + dy * dy;
	double best_progress = std::numeric_limits<double>::max();
	double best_distance = std::numeric_limits<double>::max();

	for (std::size_t index = 0; index < weeds_.size(); ++index) {
		// Distant weeds remain pending. This is the key rule that keeps the robot
		// close to the planned search route.
		if (!weed_is_covered_by_search(weeds_[index], current_pose, result.pose)) {
			continue;
		}

		// Projection gives the weed's fractional progress along this segment:
		// zero is at the robot and one is at the search waypoint.
		const double progress = segment_length_squared == 0.0 ? 0.0 :
			((weeds_[index].position.x - current_pose.pose.position.x) * dx +
			(weeds_[index].position.y - current_pose.pose.position.y) * dy) /
			segment_length_squared;
		if (progress < 0.0 || progress > 1.0) {
			// A weed near the infinite line but outside the finite segment is not
			// part of this search leg and waits for a later leg.
			continue;
		}
		const double current_distance = distance(
			current_pose.pose.position, weeds_[index].position);
		if (progress < best_progress ||
			(progress == best_progress && current_distance < best_distance)) {
			// Select the first weed encountered along the route. Distance is only
			// a tie-breaker for detections at the same route position.
			best_progress = progress;
			best_distance = current_distance;
			result.pose.pose.position = weeds_[index].position;
			result.pose.pose.orientation = current_pose.pose.orientation;
			result.weed_index = index;
		}
	}

	return result;
}

void WombatProcessing::complete_goal(const Goal & goal)
{
	if (goal.weed_index.has_value() && goal.weed_index.value() < weeds_.size()) {
		// A weed is considered handled only after Nav2 reaches it and the node
		// completes the configured inspection pause.
		weeds_.erase(weeds_.begin() + static_cast<long>(goal.weed_index.value()));
		return;
	}

	++search_segment_;
}

geometry_msgs::msg::PoseStamped WombatProcessing::next_search_waypoint() const
{
	geometry_msgs::msg::PoseStamped waypoint = origin_;
	// Two consecutive sides share a length. The square grows after each pair.
	const double side_length = initial_side_length_ +
		side_length_increment_ * static_cast<double>(search_segment_ / 2);
	const double quarter_turn = 1.5707963267948966;
	const std::size_t direction = search_segment_ % 4;

	if (direction == 0) {
		// Positive Y is north in the configured map frame.
		waypoint.pose.position.y += side_length;
	} else if (direction == 1) {
		// Positive X is east.
		waypoint.pose.position.x += side_length;
	} else if (direction == 2) {
		// Negative Y is south.
		waypoint.pose.position.y -= side_length;
	} else {
		// Negative X is west.
		waypoint.pose.position.x -= side_length;
	}

	// Nav2 uses a planar quaternion. Convert the travel direction into yaw.
	const double heading = quarter_turn * (direction == 0 ? 1.0 :
		direction == 1 ? 0.0 : direction == 2 ? -1.0 : 2.0);
	waypoint.pose.orientation.w = std::cos(heading / 2.0);
	waypoint.pose.orientation.z = std::sin(heading / 2.0);
	return waypoint;
}

bool WombatProcessing::weed_is_covered_by_search(
	const geometry_msgs::msg::Pose & weed,
	const geometry_msgs::msg::PoseStamped & current_pose,
	const geometry_msgs::msg::PoseStamped & search_goal) const
{
	// Measure distance to the finite segment, not its infinite line.
	return point_to_segment_distance(
		weed.position, current_pose.pose.position, search_goal.pose.position) <=
		search_corridor_width_;
}

double WombatProcessing::distance(
	const geometry_msgs::msg::Point & first,
	const geometry_msgs::msg::Point & second)
{
	// Ground distance is sufficient for route ordering; z does not affect
	// navigation of the ground robot.
	return std::hypot(first.x - second.x, first.y - second.y);
}

double WombatProcessing::point_to_segment_distance(
	const geometry_msgs::msg::Point & point,
	const geometry_msgs::msg::Point & segment_start,
	const geometry_msgs::msg::Point & segment_end)
{
	// Project the point onto the segment and clamp the projection to [0, 1].
	// Clamping prevents points behind the robot or beyond the waypoint from
	// being treated as part of the upcoming search side.
	const double dx = segment_end.x - segment_start.x;
	const double dy = segment_end.y - segment_start.y;
	const double length_squared = dx * dx + dy * dy;
	if (length_squared == 0.0) {
		return distance(point, segment_start);
	}

	const double projection = std::clamp(
		((point.x - segment_start.x) * dx + (point.y - segment_start.y) * dy) /
		length_squared, 0.0, 1.0);
	geometry_msgs::msg::Point nearest = segment_start;
	nearest.x += projection * dx;
	nearest.y += projection * dy;
	return distance(point, nearest);
}