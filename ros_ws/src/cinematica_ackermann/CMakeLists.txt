cmake_minimum_required(VERSION 3.8)
project(cinematica_ackermann)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cinematica_interfaces REQUIRED)

add_executable(cinematica_ackermann_node src/cinematica_ackermann_node.cpp)
ament_target_dependencies(cinematica_ackermann_node rclcpp geometry_msgs std_msgs cinematica_interfaces)

install(TARGETS cinematica_ackermann_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})



ament_package()