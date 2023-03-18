cmake_minimum_required(VERSION 3.8)
project(turtlesim_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtlesim_controller src/turtlesim_controller.cpp)

ament_target_dependencies(turtlesim_controller rclcpp turtlesim geometry_msgs rclcpp_action)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
   set(ament_cmake_copyright_FOUND TRUE)
   set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(TARGETS turtlesim_controller
  DESTINATION lib/${PROJECT_NAME})
ament_package()
