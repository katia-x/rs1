#ifndef RS1_WOMBAT_NODE_H_
#define RS1_WOMBAT_NODE_H_

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include "rs1/msg/wombat_status.hpp"
#include "rs1/msg/wombat_weed_goal.hpp"
#include "rs1/srv/wombat_next_goal.hpp"

#include <memory>
#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_set>

class WombatNode : public rclcpp::Node
{
public:
	WombatNode();

private:
	using NavigateToPose = nav2_msgs::action::NavigateToPose;
	using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
	using DeleteEntity = ros_gz_interfaces::srv::DeleteEntity;

	enum class State
	{
		WaitingForRegion,
		ApproachingStart,
		PreLaneWait,
		Turning,
		Searching,
		Recovery,
		Complete,
		Abort
	};
	struct PendingRequest
	{
		bool pending{true};
		std::uint8_t operation{rs1::srv::WombatNextGoal::Request::REQUEST_NEXT_GOAL};
		std::uint64_t weed_id{0};
	};
	struct PendingWeedGoal
	{
		bool pending{false};
		rs1::msg::WombatWeedGoal goal;
	};

	void timer_callback();
	void request_next_goal(std::uint8_t operation, std::uint64_t weed_id = 0);
	void queue_request(std::uint8_t operation, std::uint64_t weed_id = 0);
	void weed_goal_callback(const rs1::msg::WombatWeedGoal::SharedPtr message);
	void status_callback(const rs1::msg::WombatStatus::SharedPtr message);
	void complete_mission();
	bool mission_stopped() const;
	void abort_mission();
	void dispatch_pending_weed_goal();
	void handle_nav_cancel(rclcpp_action::Client<NavigateToPose>::CancelResponse::SharedPtr response);
	void send_navigation_goal(const geometry_msgs::msg::PoseStamped & pose, bool is_weed,
	std::uint64_t weed_id, const std::string & weed_name = {});
	void reset_for_new_region();
	void handle_nav_failure();
	void handle_mission_goal(rclcpp::Client<rs1::srv::WombatNextGoal>::SharedFuture future);
	void handle_nav_goal_response(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
	void handle_nav_result(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);
	void try_delete_weed();
	void finish_weed_removal(bool removed);

	const std::string global_frame_;
	const double lane_start_wait_duration_;
	const int max_goal_retries_;
	const double nav_unavailable_timeout_;

	rclcpp::Client<rs1::srv::WombatNextGoal>::SharedPtr mission_client_;
	rclcpp::Client<DeleteEntity>::SharedPtr delete_entity_client_;
	rclcpp::Subscription<rs1::msg::WombatWeedGoal>::SharedPtr weed_goal_subscription_;
	rclcpp::Subscription<rs1::msg::WombatStatus>::SharedPtr status_subscription_;
	rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr weeds_removed_publisher_;
	rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
	NavigateGoalHandle::SharedPtr active_goal_;
	NavigateGoalHandle::SharedPtr canceling_goal_;
	geometry_msgs::msg::PoseStamped last_navigation_goal_;
	bool last_navigation_goal_valid_{false};
	bool last_goal_is_weed_{false};
	std::uint64_t last_weed_id_{0};
	rclcpp::TimerBase::SharedPtr mission_retry_timer_;
	bool weed_deletion_in_flight_{false};
	bool weed_deletion_retry_pending_{false};
	int weed_deletion_attempts_{0};
	std::string last_weed_name_;
	bool mission_request_in_flight_{false};
	std::uint8_t mission_request_operation_{rs1::srv::WombatNextGoal::Request::REQUEST_NEXT_GOAL};
	bool nav_goal_request_in_flight_{false};
	bool nav_cancel_in_flight_{false};
	PendingRequest pending_request_;
	PendingWeedGoal pending_weed_goal_;
	std::size_t goal_failures_{0};
	std::uint64_t mission_generation_{0};
	std::uint64_t mission_request_generation_{0};
	State state_{State::WaitingForRegion};
	std::chrono::steady_clock::time_point lane_wait_until_{};
	bool nav_unavailable{false};
	std::chrono::steady_clock::time_point nav_unavailable_since_{};
	std::unordered_set<std::string> removed_plant_names_;
	std::uint64_t deletion_request_id_{0};
	std::chrono::steady_clock::time_point deletion_deadline_{};
	std::int64_t deletion_ros_request_id_{0};
};

#endif
