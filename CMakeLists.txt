cmake_minimum_required(VERSION 3.8)
project(rs1)

find_package(ament_cmake REQUIRED)
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

add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(
  mission_node
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rosidl_target_interfaces(mission_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(wombat_node src/wombat/wombat_node.cpp)
ament_target_dependencies(
  wombat_node
  nav2_msgs
  rclcpp
  rclcpp_action
)
rosidl_target_interfaces(wombat_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(DIRECTORY launch config worlds urdf_husky urdf_parrot
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(TARGETS
  mission_node
  wombat_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
