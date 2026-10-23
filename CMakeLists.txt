cmake_minimum_required(VERSION 3.8)
project(rs1)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/WombatNextGoal.srv"
  DEPENDENCIES geometry_msgs
)

add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(
  mission_node
  geometry_msgs
  rclcpp
  std_msgs
  vision_msgs
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

ament_add_gtest(mission_processing_test
  test/basic_tests.cpp
  src/mission_processing.cpp
)
ament_target_dependencies(mission_processing_test geometry_msgs rclcpp sensor_msgs)
target_include_directories(mission_processing_test PRIVATE src)

add_executable(plant_detector
  src/plant_perception/plant_detector.cpp
  src/plant_perception/plant_detector_main.cpp
  src/plant_perception/rtmdet.cpp
)
target_compile_features(plant_detector PRIVATE cxx_std_17)
target_include_directories(plant_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(plant_detector rclcpp sensor_msgs vision_msgs cv_bridge)
target_link_libraries(plant_detector ${OpenCV_LIBS})
install(TARGETS plant_detector DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY machine_learning/models/ DESTINATION share/${PROJECT_NAME}/machine_learning/models
  FILES_MATCHING PATTERN "*.onnx")

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
