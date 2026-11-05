#ifndef RS1_GUI_MANAGEMENT_H_
#define RS1_GUI_MANAGEMENT_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cstdint>
#include <functional>
#include <string>

class GuiManagement
{
public:
	using StateCallback = std::function<void()>;
	using OverrideCallback = std::function<void(bool)>;

	GuiManagement(
		rclcpp::Node & node,
		OverrideCallback override_callback,
		StateCallback estop_callback,
		StateCallback resume_callback);

	bool manual_override() const;
	bool estop_active() const;
	bool resume_required() const;
	std::uint64_t override_generation() const;
	std::uint64_t control_generation() const;

private:
	void override_callback(const std_msgs::msg::Bool::ConstSharedPtr message);
	void estop_callback(const std_msgs::msg::Bool::ConstSharedPtr message);
	void resume_callback(const std_msgs::msg::Bool::ConstSharedPtr message);
	void publish_override_state();
	void publish_estop_state();

	rclcpp::Node & node_;
	OverrideCallback on_override_;
	StateCallback on_estop_;
	StateCallback on_resume_;
	bool manual_override_{false};
	bool estop_active_{false};
	bool estop_release_ready_{false};
	bool resume_required_{false};
	std::uint64_t override_generation_{0};
	std::uint64_t control_generation_{0};
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr override_subscription_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr override_state_publisher_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_state_publisher_;
};

#endif
