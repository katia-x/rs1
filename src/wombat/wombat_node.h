#ifndef RS1_WOMBAT_NODE_H_
#define RS1_WOMBAT_NODE_H_

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
		GoingToWeed,
		Recovery,
		Complete
	};

	void timer_callback();
	void request_next_goal(bool skip_current_goal = false, bool reassess = false,
		bool weed_handled = false);
	void queue_request(bool skip_current_goal, bool reassess, bool weed_handled = false);
	void cancel_for_reassessment();
	void handle_mission_goal(rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future);
	void handle_nav_goal_response(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
	void handle_nav_result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
	void schedule_retry(bool skip_current_goal);

	const std::string global_frame_;
	const double lane_start_wait_duration_;

	using NavigateToPose = nav2_msgs::action::NavigateToPose;
	using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

	rclcpp::Client<rs1::srv::WombatNextGoal>::SharedPtr mission_client_;
	rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
	NavigateGoalHandle::SharedPtr active_goal_;
	geometry_msgs::msg::PoseStamped last_goal_;
	bool last_goal_valid_{false};
	bool last_goal_is_weed_{false};
	std::uint64_t last_weed_id_{0};
	rclcpp::TimerBase::SharedPtr mission_retry_timer_;
	bool mission_request_in_flight_{false};
	bool request_pending_{true};
	bool pending_skip_current_goal_{false};
	bool pending_reassess_{false};
	bool pending_weed_handled_{false};
	bool cancel_in_flight_{false};
	bool intentional_cancel_{false};
	State state_{State::WaitingForRegion};
	std::chrono::steady_clock::time_point lane_wait_until_{};
};

#endif