cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(my_robot_controller)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  ament_export_dependencies(ament_lint_auto)
  ament_export_dependencies(ament_flake8)
  ament_export_dependencies(ament_pep257)
  ament_export_dependencies(python3-pytest)
endif()
