cmake_minimum_required(VERSION 3.8)
project(rs1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Glider processing library (pure logic, no ROS)
# ---------------------------------------------------------------------------
add_library(glider_processing
  src/glider/glider_processing.cpp
)
target_include_directories(glider_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Glider node (ROS communication only)
# ---------------------------------------------------------------------------
add_executable(glider_node
  src/glider/glider_node.cpp
)
target_include_directories(glider_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(glider_node glider_processing)
ament_target_dependencies(glider_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  glider_processing
  glider_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config worlds urdf_husky urdf_parrot
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()