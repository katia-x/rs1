cmake_minimum_required(VERSION 3.8)
project(rs1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(geometry_msgs REQUIRED)li
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mission_node
  src/main.cpp
  src/mission_node.cpp
  src/mission_processing.cpp
)
ament_target_dependencies(mission_node geometry_msgs rclcpp sensor_msgs)

install(TARGETS mission_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_add_gtest(mission_processing_test
  test/basic_tests.cpp
  src/mission_processing.cpp
)
ament_target_dependencies(mission_processing_test geometry_msgs rclcpp sensor_msgs)
target_include_directories(mission_processing_test PRIVATE src)

install(DIRECTORY launch config worlds urdf_husky
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
