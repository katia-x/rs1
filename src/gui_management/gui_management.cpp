#include "gui_management.h"

#include <functional>
#include <utility>

GuiManagement::GuiManagement(
	rclcpp::Node & node,
	OverrideCallback override_callback,
	StateCallback estop_callback,
	StateCallback resume_callback)
: node_(node),
  on_override_(std::move(override_callback)),
  on_estop_(std::move(estop_callback)),
  on_resume_(std::move(resume_callback))
{
	const auto override_command_topic = node_.declare_parameter<std::string>(
		"manual_override_command_topic", "/husky1/manual_override_command");
	const auto override_state_topic = node_.declare_parameter<std::string>(
		"manual_override_state_topic", "/husky1/manual_override");
	const auto estop_command_topic = node_.declare_parameter<std::string>(
		"estop_command_topic", "/husky1/estop_command");
	const auto estop_state_topic = node_.declare_parameter<std::string>(
		"estop_state_topic", "/husky1/estop");
	const auto resume_command_topic = node_.declare_parameter<std::string>(
		"resume_command_topic", "/husky1/resume_command");

	override_state_publisher_ = node_.create_publisher<std_msgs::msg::Bool>(
		override_state_topic, rclcpp::QoS(1).transient_local());
	override_subscription_ = node_.create_subscription<std_msgs::msg::Bool>(
		override_command_topic, rclcpp::QoS(10),
		std::bind(&GuiManagement::override_callback, this, std::placeholders::_1));
	estop_state_publisher_ = node_.create_publisher<std_msgs::msg::Bool>(
		estop_state_topic, rclcpp::QoS(1).transient_local());
	estop_subscription_ = node_.create_subscription<std_msgs::msg::Bool>(
		estop_command_topic, rclcpp::QoS(10),
		std::bind(&GuiManagement::estop_callback, this, std::placeholders::_1));
	resume_subscription_ = node_.create_subscription<std_msgs::msg::Bool>(
		resume_command_topic, rclcpp::QoS(10),
		std::bind(&GuiManagement::resume_callback, this, std::placeholders::_1));
	publish_override_state();
	publish_estop_state();
}

bool GuiManagement::manual_override() const
{
	return manual_override_;
}

bool GuiManagement::estop_active() const
{
	return estop_active_;
}

bool GuiManagement::resume_required() const
{
	return resume_required_;
}

std::uint64_t GuiManagement::override_generation() const
{
	return override_generation_;
}

void GuiManagement::override_callback(const std_msgs::msg::Bool::ConstSharedPtr message)
{
	if (!message || message->data == manual_override_) {
		return;
	}
	if (!message->data && estop_active_) {
		RCLCPP_WARN(node_.get_logger(), "Ignoring automatic-mode request while E-stop is active");
		return;
	}
	if (!message->data && resume_required_) {
		RCLCPP_WARN(node_.get_logger(), "Ignoring automatic-mode request; press Resume after releasing E-stop");
		return;
	}
	manual_override_ = message->data;
	++override_generation_;
	if (manual_override_) {
		if (on_override_) {
			on_override_(true);
		}
		RCLCPP_WARN(node_.get_logger(),
			"Manual override enabled; autonomous navigation stopped; Teleop may drive /husky1/cmd_vel");
	} else {
		RCLCPP_INFO(node_.get_logger(), "Manual override disabled; autonomous navigation may resume");
		if (on_override_) {
			on_override_(false);
		}
	}
	publish_override_state();
}

void GuiManagement::estop_callback(const std_msgs::msg::Bool::ConstSharedPtr message)
{
	if (!message || message->data == estop_active_) {
		return;
	}
	estop_active_ = message->data;
	++override_generation_;
	if (estop_active_) {
		estop_release_ready_ = false;
		resume_required_ = true;
		if (on_estop_) {
			on_estop_();
		}
		RCLCPP_ERROR(node_.get_logger(), "E-stop enabled; autonomous navigation stopped");
	} else {
		estop_release_ready_ = true;
		RCLCPP_WARN(node_.get_logger(), "E-stop released; press Resume to re-enable autonomous navigation");
	}
	publish_estop_state();
}

void GuiManagement::resume_callback(const std_msgs::msg::Bool::ConstSharedPtr message)
{
	if (!message || !message->data || estop_active_ || !estop_release_ready_ || !resume_required_) {
		return;
	}
	resume_required_ = false;
	manual_override_ = false;
	++override_generation_;
	RCLCPP_INFO(node_.get_logger(), "Resume accepted; autonomous navigation may resume");
	publish_override_state();
	if (on_resume_) {
		on_resume_();
	}
}

void GuiManagement::publish_override_state()
{
	std_msgs::msg::Bool state;
	state.data = manual_override_;
	override_state_publisher_->publish(state);
}

void GuiManagement::publish_estop_state()
{
	std_msgs::msg::Bool state;
	state.data = estop_active_;
	estop_state_publisher_->publish(state);
}
