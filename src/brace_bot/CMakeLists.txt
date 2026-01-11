cmake_minimum_required(VERSION 3.5)
project(brace_bot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(brace_state_node src/brace_state_node.cpp)

ament_target_dependencies(brace_state_node
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
)

# 链接系统中的 yaml-cpp 库
target_link_libraries(brace_state_node yaml-cpp)

install(TARGETS brace_state_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/brace_ui_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
