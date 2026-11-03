#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mission_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionNode>());
    rclcpp::shutdown();
    return 0;
}