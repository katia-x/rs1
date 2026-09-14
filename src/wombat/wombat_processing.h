#ifndef RS1_WOMBAT_PROCESSING_H_
#define RS1_WOMBAT_PROCESSING_H_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cstddef>
#include <optional>
#include <vector>

class WombatProcessing
{
public:
	// A selected goal is either a search waypoint or a weed inspection point.
	// weed_index is present only for a weed goal, allowing the node to decide
	// whether to advance the search or remove a weed after inspection.
	struct Goal
	{
		geometry_msgs::msg::PoseStamped pose;
		std::optional<std::size_t> weed_index;
	};

	WombatProcessing(double initial_side_length, double side_length_increment, double search_corridor_width);

	// The first robot pose becomes the fixed centre of the expanding square.
	// Later poses do not replace it, otherwise the search pattern would drift.
	void set_origin(const geometry_msgs::msg::PoseStamped & origin);
	bool has_origin() const;

	// Replace the current detection snapshot. Duplicate handling belongs to the
	// perception or mission layer, so this class plans from the latest array.
	void update_weeds(const std::vector<geometry_msgs::msg::Pose> & weeds);

	// Select the next action using the current robot pose and pending detections.
	Goal choose_goal(const geometry_msgs::msg::PoseStamped & current_pose);

	// Advance the search after a search waypoint, or remove a weed after its
	// inspection pause has completed.
	void complete_goal(const Goal & goal);

private:
	// Generate the next side of the square in north, east, south, west order.
	geometry_msgs::msg::PoseStamped next_search_waypoint() const;

	// Test whether a weed lies within the corridor around the next search side.
	bool weed_is_covered_by_search(
		const geometry_msgs::msg::Pose & weed,
		const geometry_msgs::msg::PoseStamped & current_pose,
		const geometry_msgs::msg::PoseStamped & search_goal) const;
	static double distance(const geometry_msgs::msg::Point & first, const geometry_msgs::msg::Point & second);
	static double point_to_segment_distance(
		const geometry_msgs::msg::Point & point,
		const geometry_msgs::msg::Point & segment_start,
		const geometry_msgs::msg::Point & segment_end);

	// Search geometry is expressed in metres in the global planning frame.
	double initial_side_length_;
	double side_length_increment_;
	double search_corridor_width_;
	geometry_msgs::msg::PoseStamped origin_;
	bool has_origin_{false};
	// Each completed search side increments this counter. Integer division by
	// two grows the side length after every pair of sides.
	std::size_t search_segment_{0};
	std::vector<geometry_msgs::msg::Pose> weeds_;
};

#endif