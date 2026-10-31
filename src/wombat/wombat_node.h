#ifndef RS1_WOMBAT_NODE_H_
#define RS1_WOMBAT_NODE_H_

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rs1/srv/wombat_next_goal.hpp"

#include <memory>
#include <chrono>
#include <string>

class WombatNode : public rclcpp::Node
{
public:
	WombatNode();

private:
	enum class State
	{
		WaitingForRegion,
		ApproachingStart,
		PreLaneWait,
		Turning,
		Searching,
		Recovery,
		Complete
	};
	struct PendingRequest
	{
		bool pending{true};
		bool skip_current_goal{false};
		bool retry_current_goal{false};
		bool reassess{false};
		bool weed_handled{false};
		bool weed_unreachable{false};
		std::uint64_t unreachable_weed_id{0};
	};

	void timer_callback();
	void request_next_goal(bool skip_current_goal = false, bool reassess = false,
		bool weed_handled = false, bool retry_current_goal = false,
		bool weed_unreachable = false, std::uint64_t unreachable_weed_id = 0);
	void queue_request(bool skip_current_goal, bool reassess, bool weed_handled = false,
		bool retry_current_goal = false, bool weed_unreachable = false,
		std::uint64_t unreachable_weed_id = 0);
	void handle_nav_failure();
	void handle_mission_goal(rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future);
	void handle_nav_goal_response(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
	void handle_nav_result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);

	const std::string global_frame_;
	const std::string navigation_behavior_tree_;
	const double lane_start_wait_duration_;
	const double reassessment_frequency_;
	const int max_goal_retries_;

	using NavigateToPose = nav2_msgs::action::NavigateToPose;
	using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

	rclcpp::Client<rs1::srv::WombatNextGoal>::SharedPtr mission_client_;
	rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
	NavigateGoalHandle::SharedPtr active_goal_;
	bool last_goal_is_weed_{false};
	std::uint64_t last_weed_id_{0};
	rclcpp::TimerBase::SharedPtr mission_retry_timer_;
	bool mission_request_in_flight_{false};
	bool mission_request_retry_{false};
	bool mission_request_reassessment_{false};
	bool nav_goal_request_in_flight_{false};
	PendingRequest pending_request_;
	std::size_t goal_failures_{0};
	State state_{State::WaitingForRegion};
	std::chrono::steady_clock::time_point lane_wait_until_{};
};

#endif