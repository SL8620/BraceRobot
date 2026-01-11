cmake_minimum_required(VERSION 3.8)
project(kvaser_motor_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

add_executable(motor_keyboard_node
  src/arm_control.cpp
  src/kvaser.cpp
)

ament_target_dependencies(motor_keyboard_node
  rclcpp
  sensor_msgs
)

target_link_libraries(motor_keyboard_node
  canlib     
)

install(TARGETS
  motor_keyboard_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

