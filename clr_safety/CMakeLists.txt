cmake_minimum_required(VERSION 3.5)
project(clr_safety)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_controller_stopper test/test_controller_stopper.py)
endif()


ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
