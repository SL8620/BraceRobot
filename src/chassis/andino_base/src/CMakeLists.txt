add_library(motor_driver SHARED
  motor_driver.cpp
  kvaser.cpp
)

target_include_directories(motor_driver
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${SERIAL_INCLUDE_DIRS}
    /usr/include/kvaser  # ✅ 可选：Kvaser SDK 的头文件路径（如有需要）
)

# ✅ 链接 libcanlib.so （Kvaser 官方库）
target_link_libraries(motor_driver
  PUBLIC
    ${SERIAL_LDFLAGS}
    ${CMAKE_THREAD_LIBS_INIT}
    canlib              # ⬅️ 关键：链接 Kvaser CANlib SDK
)

# diffdrive_andino 保持不变
add_library(diffdrive_andino SHARED
  diffdrive_andino.cpp
  wheel.cpp
)

target_include_directories(diffdrive_andino
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(diffdrive_andino
  PUBLIC
    hardware_interface
    pluginlib
    rclcpp
)

target_link_libraries(diffdrive_andino PUBLIC motor_driver)

install(
  TARGETS diffdrive_andino motor_driver
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(diffdrive_andino motor_driver)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp)
