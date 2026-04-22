cmake_minimum_required(VERSION 3.8)
project(hydronaut_description)

find_package(ament_cmake REQUIRED)

# Copy the important folders to the install directory
install(DIRECTORY
  launch
  meshes
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
