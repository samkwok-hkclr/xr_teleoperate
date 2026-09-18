cmake_minimum_required(VERSION 3.16)
project(dual_arm_hardware_interface LANGUAGES CXX)

# ============================================================
# 基础设置
# ============================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# 默认构建类型（单配置生成器）
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      Debug Release RelWithDebInfo MinSizeRel)
  message(STATUS "[dual_arm_hardware_interface] "
                 "CMAKE_BUILD_TYPE not set, defaulting to Release")
endif()

# ============================================================
# 依赖
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# SDK 包（ROS2 模式由 ament 提供）
find_package(dual_arm_v2_2_control_sdk REQUIRED)

# ============================================================
# 库目标
# ============================================================
add_library(dual_arm_hardware_interface SHARED
  # ---- 基础设施 ----
  src/control_api_manager.cpp
  src/hardware_base.cpp

  # ---- 组件抽象层 ----
  src/joint_group_base.cpp
  src/gripper_base.cpp

  # ---- 具体实现 ----
  src/left_arm_hardware_interface.cpp
  src/right_arm_hardware_interface.cpp
  src/head_hardware_interface.cpp
  src/elevator_hardware_interface.cpp
  src/left_gripper_hardware_interface.cpp
  src/right_gripper_hardware_interface.cpp
)

target_include_directories(dual_arm_hardware_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(dual_arm_hardware_interface
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    lifecycle_msgs::lifecycle_msgs
    dual_arm_v2_2_control_sdk::dual_arm_v2_2_control_sdk
  PRIVATE
    diagnostic_updater::diagnostic_updater
)

# ============================================================
# 编译选项（仅作用于本 target）
# ============================================================
target_compile_options(dual_arm_hardware_interface PRIVATE
  # 警告
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall>
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wextra>
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wpedantic>

  # Debug
  $<$<CONFIG:Debug>:-O0;-g3;-fno-omit-frame-pointer>

  # Release —— 显式补回 -DNDEBUG
  $<$<CONFIG:Release>:-O2;-DNDEBUG>

  # RelWithDebInfo
  $<$<CONFIG:RelWithDebInfo>:-O2;-g;-DNDEBUG>

  # MinSizeRel
  $<$<CONFIG:MinSizeRel>:-Os;-DNDEBUG>
)

# ============================================================
# pluginlib 导出
# ============================================================
pluginlib_export_plugin_description_file(
  hardware_interface dual_arm_hardware_interface_plugin.xml)

# ============================================================
# 安装
# ============================================================
install(TARGETS dual_arm_hardware_interface
  EXPORT  export_dual_arm_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include)

# ============================================================
# ament 导出
# ============================================================
ament_export_targets(export_dual_arm_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  lifecycle_msgs
  diagnostic_updater
  dual_arm_v2_2_control_sdk
)

ament_package()