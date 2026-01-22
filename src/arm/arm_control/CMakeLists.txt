cmake_minimum_required(VERSION 3.8)
project(kvaser_motor_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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
  std_msgs
)

target_link_libraries(motor_keyboard_node
  canlib     
)

add_executable(motor5_pt_impedance_node
  src/motor5_pt_impedance_node.cpp
  src/kvaser.cpp
)

ament_target_dependencies(motor5_pt_impedance_node
  rclcpp
  std_msgs
)

target_link_libraries(motor5_pt_impedance_node
  canlib
)

add_executable(arm_pt_impedance_node
  src/arm_pt_impedance_node.cpp
  src/kvaser.cpp
)

ament_target_dependencies(arm_pt_impedance_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(arm_pt_impedance_node
  canlib
)

install(TARGETS
  motor_keyboard_node
  motor5_pt_impedance_node
  arm_pt_impedance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

