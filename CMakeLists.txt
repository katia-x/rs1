cmake_minimum_required(VERSION 3.5)
project(rs1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlantDetection.msg"
  "msg/PlantDetectionArray.msg"
  "srv/WombatNextGoal.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(wombat_node src/wombat/wombat_node.cpp)
ament_target_dependencies(
  wombat_node
  rclcpp
  nav2_msgs
  rclcpp_action
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
rosidl_target_interfaces(wombat_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(
  mission_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
rosidl_target_interfaces(mission_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  wombat_node
  mission_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS
      launch/wombat_launch.py
    DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()