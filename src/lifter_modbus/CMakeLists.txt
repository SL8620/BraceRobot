cmake_minimum_required(VERSION 3.8)
project(lifter_modbus)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)

add_library(lifter_modbus_lib
  src/lifter.cpp
)

ament_target_dependencies(lifter_modbus_lib
  ament_index_cpp
)

target_include_directories(lifter_modbus_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(lifter_node
  src/lifter_node.cpp
)

ament_target_dependencies(lifter_node
  rclcpp
  std_msgs
)

target_link_libraries(lifter_node
  lifter_modbus_lib
)

add_executable(lifter_test
  src/lifter_test.cpp
)

ament_target_dependencies(lifter_test
  rclcpp
)

target_link_libraries(lifter_test
  lifter_modbus_lib
)

install(TARGETS
  lifter_modbus_lib
  lifter_node
  lifter_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/lifter_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()