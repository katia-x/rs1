cmake_minimum_required(VERSION 3.5)
project(rs1)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

## Find packages, remove those that are not needed
find_package(ament_cmake REQUIRED)
<<<<<<< Updated upstream
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)

#################################################################

# I HAVE COPIED OVER MY CMAKE FILE FROM PFMS, THIS WILL NEED TO BE CHANGED TO ALIGN WITH RS1 IMPLEMENTATION

#################################################################
# The library we are developing will be called laserprocessing
add_library(laserprocessing src/laserprocessing.cpp)
ament_target_dependencies(laserprocessing rclcpp std_msgs sensor_msgs nav_msgs)

add_library(controllerprocessing src/controllerProcessing.cpp)
ament_target_dependencies(controllerprocessing geometry_msgs nav_msgs)

add_library(missionprocessing src/missionProcessing.cpp)
ament_target_dependencies(missionprocessing geometry_msgs nav_msgs)
=======
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

ament_add_gtest(mission_processing_test
  test/basic_tests.cpp
  src/mission_processing.cpp
)
ament_target_dependencies(mission_processing_test geometry_msgs rclcpp sensor_msgs)
target_include_directories(mission_processing_test PRIVATE src)

install(TARGETS mission_node
  DESTINATION lib/${PROJECT_NAME}
)
>>>>>>> Stashed changes

add_library(trackmap src/trackMap.cpp)
ament_target_dependencies(trackmap geometry_msgs nav_msgs sensor_msgs)

add_library(centrelinemap src/centrelineMap.cpp)
ament_target_dependencies(centrelinemap geometry_msgs nav_msgs)

add_library(racingline src/racingLine.cpp)
ament_target_dependencies(racingline geometry_msgs nav_msgs)
target_link_libraries(racingline trackmap)

## This is the mission node executable
add_executable(mission_node src/main_mission.cpp src/mission_node.cpp)
ament_target_dependencies(mission_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  std_srvs 
  visualization_msgs
  geometry_msgs
)
target_link_libraries(mission_node laserprocessing missionprocessing trackmap centrelinemap racingline)

## This is the controller node executable
add_executable(controller_node src/main_controller.cpp src/controller_node.cpp)
ament_target_dependencies(controller_node 
  rclcpp 
  std_msgs 
  nav_msgs 
  geometry_msgs
)
target_link_libraries(controller_node controllerprocessing)

# ## This is the combined executable for PFMS A3
add_executable(both_nodes src/main_both.cpp src/mission_node.cpp src/controller_node.cpp)
ament_target_dependencies(both_nodes 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  std_srvs
  visualization_msgs
  geometry_msgs
)
target_link_libraries(both_nodes laserprocessing controllerprocessing missionprocessing trackmap centrelinemap racingline)

# ## This is the multiplayer executable for all 3 nodes
add_executable(multiplayer_nodes
  src/main_multiplayer.cpp
  src/mission_node.cpp
  src/controller_node.cpp
  src/multiplayer_node.cpp
)
ament_target_dependencies(multiplayer_nodes
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  visualization_msgs
  geometry_msgs
)
target_link_libraries(multiplayer_nodes laserprocessing controllerprocessing missionprocessing trackmap centrelinemap racingline)

install(TARGETS
  mission_node
  controller_node
  both_nodes
  multiplayer_nodes
  laserprocessing
  controllerprocessing
  trackmap
  centrelinemap
  racingline
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    data
  DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY
      launch
    DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

  ament_add_ros_isolated_gtest(test_basic test/basic_tests.cpp)
  ament_add_ros_isolated_gtest(test_advanced test/advanced_tests.cpp)
  ament_add_ros_isolated_gtest(test_trackmap test/trackmap_tests.cpp)
  ament_add_ros_isolated_gtest(test_centrelinemap test/centrelinemap_tests.cpp)
  ament_add_ros_isolated_gtest(test_racingline test/racingline_tests.cpp)
  ament_add_ros_isolated_gtest(test_controllerprocessing test/controllerprocessing_tests.cpp)
  ament_add_ros_isolated_gtest(
    test_node_lifecycle
    test/node_lifecycle_tests.cpp
    src/mission_node.cpp
    src/controller_node.cpp
  )

  target_link_libraries(test_basic laserprocessing)
  target_link_libraries(test_advanced laserprocessing missionprocessing)
  target_link_libraries(test_trackmap trackmap)
  target_link_libraries(test_centrelinemap centrelinemap)
  target_link_libraries(test_racingline racingline trackmap)
  target_link_libraries(test_controllerprocessing controllerprocessing)
  target_link_libraries(
    test_node_lifecycle
    laserprocessing
    controllerprocessing
    missionprocessing
    trackmap
    centrelinemap
    racingline
  )

  ament_target_dependencies(test_basic rclcpp sensor_msgs nav_msgs rosbag2_cpp ament_index_cpp)
  ament_target_dependencies(test_advanced rclcpp sensor_msgs nav_msgs geometry_msgs rosbag2_cpp ament_index_cpp)
  ament_target_dependencies(test_trackmap sensor_msgs nav_msgs geometry_msgs)
  ament_target_dependencies(test_centrelinemap nav_msgs geometry_msgs)
  ament_target_dependencies(test_racingline sensor_msgs nav_msgs geometry_msgs)
  ament_target_dependencies(test_controllerprocessing nav_msgs geometry_msgs)
  ament_target_dependencies(
    test_node_lifecycle
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    std_srvs
    visualization_msgs
    geometry_msgs
  )

endif()

ament_package()