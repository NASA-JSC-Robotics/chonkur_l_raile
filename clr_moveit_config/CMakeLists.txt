cmake_minimum_required(VERSION 3.5)
project(clr_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/enable_collision_checking.py
  scripts/disable_collision_checking.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
