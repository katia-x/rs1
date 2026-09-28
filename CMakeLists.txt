cmake_minimum_required(VERSION 3.8)
project(rs1)

find_package(ament_cmake REQUIRED)

# Simulation-only bringup. Add C++ targets when the controller scaffold has
# implementations and executable entry points.
install(DIRECTORY launch config worlds urdf_husky urdf_parrot
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
