cmake_minimum_required(VERSION 3.8)
project(rs1)

find_package(ament_cmake REQUIRED)

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
install(DIRECTORY launch config worlds urdf_husky urdf_parrot
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
