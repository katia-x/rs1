// platform.h
#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>

class Platform
{
public:
  explicit Platform(const rclcpp::Logger & logger);
  void spawnPoo(const std::string & name, double x, double y, double z);

private:
  rclcpp::Logger logger_;
};