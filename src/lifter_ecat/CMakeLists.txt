cmake_minimum_required(VERSION 3.10)
project(lifter_ecat)

# 查找 ROS2 依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# 设置可执行文件
add_executable(lifter_node
  src/lifter_node.cpp
  src/lifter.cpp
)

# 链接 yaml-cpp
target_link_libraries(lifter_node
  yaml-cpp
)

# 让编译器能找到 include/lifter_ecat/lifter.hpp
target_include_directories(lifter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ===== SOEM: prefer find_package(soem CONFIG), fallback to local build/install =====
find_package(soem CONFIG QUIET)

if(soem_FOUND)
  message(STATUS "Found SOEM via find_package")
  if(TARGET soem::soem)
    set(SOEM_LIB_TARGET soem::soem)
  elseif(TARGET soem)
    set(SOEM_LIB_TARGET soem)
  else()
    set(SOEM_LIB_TARGET soem)
  endif()
else()
  message(WARNING "SOEM not found via find_package; falling back to local SOEM build/install")
  set(SOEM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../SOEM/build/install" CACHE PATH "Local SOEM install")
  if(EXISTS "${SOEM_ROOT}/lib/libsoem.a")
    add_library(soem_static STATIC IMPORTED)
    set_target_properties(soem_static PROPERTIES
      IMPORTED_LOCATION "${SOEM_ROOT}/lib/libsoem.a"
      INTERFACE_INCLUDE_DIRECTORIES "${SOEM_ROOT}/include"
    )
    set(SOEM_LIB_TARGET soem_static)
  else()
    message(FATAL_ERROR "SOEM not found: neither find_package(soem) nor local ${SOEM_ROOT}/lib/libsoem.a exist")
  endif()
endif()

# 确保目标的包含目录对节点可见
target_include_directories(lifter_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<TARGET_PROPERTY:${SOEM_LIB_TARGET},INTERFACE_INCLUDE_DIRECTORIES>
)

# 链接 SOEM 静态/导出目标，以及系统库
target_link_libraries(lifter_node
  ${SOEM_LIB_TARGET}
  pthread
  rt
)

# 确保已安装的可执行文件可以在运行时找到 ROS 和工作空间库
set_target_properties(lifter_node PROPERTIES
  BUILD_RPATH "$ORIGIN/../lib:/opt/ros/humble/lib:$ORIGIN/../../lib"
  INSTALL_RPATH "$ORIGIN/../lib:/opt/ros/humble/lib:$ORIGIN/../../lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# ROS2 依赖项
ament_target_dependencies(lifter_node
  rclcpp
  std_msgs
  ament_index_cpp
  yaml-cpp
)

# 安装可执行文件
install(TARGETS lifter_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（可选但建议）
install(DIRECTORY include/
  DESTINATION include
)

# 安装 config/launch（建议）
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
