cmake_minimum_required(VERSION 3.8)
project(yangsim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yinyang_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


add_executable(yangnode src/yangnode.cpp)
ament_target_dependencies(yangnode rclcpp yinyang_msgs rclcpp_action)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(TARGETS yangnode
  DESTINATION lib/${PROJECT_NAME})
ament_package()
