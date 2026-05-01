cmake_minimum_required(VERSION 3.16)
project(new_mobile_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
