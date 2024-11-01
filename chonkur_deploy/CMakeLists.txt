cmake_minimum_required(VERSION 3.5)
project(chonkur_deploy)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
    scripts/chonkur_controller_stopper.py
    scripts/dummy_program_state_server.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
