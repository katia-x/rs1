#include "plant_detector.h"

#include <exception>
#include <iostream>
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<PlantDetector>();
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 2);
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception& error) {
        std::cerr << "Plant detector startup failed: " << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
