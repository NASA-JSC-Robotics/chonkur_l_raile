cmake_minimum_required(VERSION 3.16.3)
project(clr_imetro_environments)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
