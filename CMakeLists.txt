cmake_minimum_required(VERSION 3.8)
project(rs1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(geometry_msgs REQUIRED)
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

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)

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
  
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#generate the cube centre hpp file
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCubeCentre.srv"
  DEPENDENCIES geometry_msgs
)

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
  nav_msgs
)
rosidl_target_interfaces(glider_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---------------------------------------------------------------------------
# Sonar node 
# ---------------------------------------------------------------------------
add_executable(sonar_node
  src/sensing/sonar.cpp
)

target_include_directories(sonar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sonar_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(sonar_node "${cpp_typesupport_target}")

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  glider_processing
  glider_node
  sonar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config worlds urdf_husky urdf_parrot
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()