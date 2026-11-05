cmake_minimum_required(VERSION 3.8)
project(rs1)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GliderState.msg"
  "msg/WombatWeedGoal.msg"
  "msg/WombatStatus.msg"
  "msg/WombatState.msg"
  "srv/WombatNextGoal.srv"
  "srv/GetCubeCentre.srv"
  "msg/GliderGoal.msg"
  DEPENDENCIES geometry_msgs
)

add_executable(mission_node src/main.cpp src/mission_node.cpp src/mission_processing.cpp)
target_compile_features(mission_node PRIVATE cxx_std_17)
target_include_directories(mission_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(mission_node ${OpenCV_LIBS})
ament_target_dependencies(
  mission_node
  cv_bridge
  geometry_msgs
  rclcpp
  rcl_interfaces
  std_msgs
  vision_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rosidl_target_interfaces(mission_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(wombat_node
  src/wombat/wombat_node.cpp
  src/gui_management/gui_management.cpp
)
target_include_directories(wombat_node PRIVATE src)
ament_target_dependencies(
  wombat_node
  action_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
)
rosidl_target_interfaces(wombat_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(weed_override_layer SHARED
  src/weed_override_layer.cpp
)
target_include_directories(weed_override_layer PRIVATE src)
ament_target_dependencies(
  weed_override_layer
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  rclcpp
  tf2_geometry_msgs
  tf2_ros
)

# --- Glider processing library ---
add_library(glider_processing
  src/glider/glider_processing.cpp
)
target_include_directories(glider_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# --- Platform library (shared poo spawning) ---
add_library(platform
  src/platform.cpp
)
target_include_directories(platform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(platform rclcpp ament_index_cpp)

# --- Glider node ---
add_executable(glider_node
  src/glider/glider_node.cpp
)
target_include_directories(glider_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(glider_node glider_processing platform)
ament_target_dependencies(glider_node
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs
)
rosidl_target_interfaces(glider_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- Sonar node ---
add_executable(sonar_node
  src/sensing/sonar.cpp
)
target_include_directories(sonar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sonar_node
  rclcpp geometry_msgs sensor_msgs nav_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(sonar_node "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mission_processing_test
    test/weed_estimation_tests.cpp
    src/mission_processing.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  if(TARGET mission_processing_test)
    target_compile_features(mission_processing_test PRIVATE cxx_std_17)
    ament_target_dependencies(mission_processing_test cv_bridge geometry_msgs rclcpp sensor_msgs vision_msgs tf2 tf2_geometry_msgs tf2_ros)
    target_include_directories(mission_processing_test PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(mission_processing_test ${OpenCV_LIBS})
    target_include_directories(mission_processing_test PRIVATE src)
  endif()

  ament_add_test(weed_target_integration
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/weed_target_integration.py"
      "$<TARGET_FILE:mission_node>" "${CMAKE_CURRENT_SOURCE_DIR}/config/mission_params.yaml"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
    APPEND_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 40
  )
endif()

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
install(TARGETS weed_override_layer
  LIBRARY DESTINATION lib
)
pluginlib_export_plugin_description_file(nav2_costmap_2d weed_layer_plugin.xml)

install(DIRECTORY behavior_trees launch config worlds urdf_husky urdf_parrot
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(TARGETS
  mission_node
  wombat_node
  glider_node
  sonar_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
