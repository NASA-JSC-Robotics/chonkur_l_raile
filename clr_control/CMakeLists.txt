cmake_minimum_required(VERSION 3.5)
project(clr_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rclpy REQUIRED)


# telling python to install modules as if they were a pip package (e.g if we wanna use classes I make in other places)
ament_python_install_package(${PROJECT_NAME})

# double check if need launch file? 
# install(
#   DIRECTORY config launch
#   DESTINATION share/${PROJECT_NAME}
# )

# this makes estop_safety an exeutable 
install(PROGRAMS 
    scripts/estop_safety.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()