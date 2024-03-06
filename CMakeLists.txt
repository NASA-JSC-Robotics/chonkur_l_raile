cmake_minimum_required(VERSION 3.5)
project(drt_ros2_control_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
ament_python_install_module(${PROJECT_NAME}/ctrl_config_compiler.py)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()