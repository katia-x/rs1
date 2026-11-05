#include "platform.h"

#include <cstdlib>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

Platform::Platform(const rclcpp::Logger & logger)
: logger_(logger)
{
}

void Platform::spawnPoo(const std::string & name, double x, double y, double z)
{
  std::string sdf_path =
    ament_index_cpp::get_package_share_directory("rs1") +
    "/worlds/poo_cube/poo_cube.sdf";

  std::stringstream ss;
  ss << "ros2 run ros_gz_sim create "
     << "-world rs1_terrain "
     << "-name " << name << " "
     << "-x " << x << " -y " << y << " -z " << z << " "
     << "-file " << sdf_path;

  int ret = std::system(ss.str().c_str());
  if (ret != 0) {
    RCLCPP_WARN(logger_, "Failed to spawn model '%s' at (%.2f, %.2f, %.2f)",
                name.c_str(), x, y, z);
  } else {
    RCLCPP_INFO(logger_, "Spawned model '%s' at (%.2f, %.2f, %.2f)",
                name.c_str(), x, y, z);
  }
}