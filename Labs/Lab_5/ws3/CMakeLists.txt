cmake_minimum_required(VERSION 3.22)
project(ws3)

find_package(ament_cmake REQUIRED)
#find_package(amnet_cmake_python REQUIRED)
#find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

ament_package()

#amnet_python_install_package(${PROJECT_NAME})

find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

add_executable(my_node ws3/node.cpp)
#add_executable(my_node2 ws3/node2.cpp)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})


ament_target_dependencies(
  my_node
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

#install(PROGRAMS
#    ws3/my_node.py
#    ws3/service.py
#    DESTINATION lib/${PROJECT_NAME}
#)