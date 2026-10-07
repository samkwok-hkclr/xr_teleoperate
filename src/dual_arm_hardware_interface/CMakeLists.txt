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
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(Boost REQUIRED)

# ament_index_cpp —— 用于解析 share/dual_arm_hardware_interface/config/zenoh.json5
find_package(ament_index_cpp REQUIRED)

# 旧 SDK（UDP ControlApi）—— 保留以兼容旧 DualArmsHardwareInterface
find_package(dual_arm_v2_2_control_sdk REQUIRED)

# ============================================================
# 包内 telemanip C ABI（Zenoh SDK）—— 作为 IMPORTED target
# ============================================================
set(TELEMANIP_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/libtelemanip_capi_cuarm_2_2.so")
set(TELEMANIP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/capi")

if(NOT EXISTS "${TELEMANIP_LIB_PATH}")
  message(FATAL_ERROR
    "[dual_arm_hardware_interface] telemanip shared library not found:\n"
    "  ${TELEMANIP_LIB_PATH}\n"
    "Place libtelemanip_capi_cuarm_2_2.so under <pkg>/lib/.")
endif()

if(NOT EXISTS "${TELEMANIP_INCLUDE_DIR}/telemanip_control.h")
  message(FATAL_ERROR
    "[dual_arm_hardware_interface] telemanip headers not found:\n"
    "  ${TELEMANIP_INCLUDE_DIR}/telemanip_control.h\n"
    "Place the capi headers under <pkg>/include/capi/.")
endif()

add_library(telemanip_capi_cuarm_2_2 SHARED IMPORTED GLOBAL)
set_target_properties(telemanip_capi_cuarm_2_2 PROPERTIES
  IMPORTED_LOCATION             "${TELEMANIP_LIB_PATH}"
  INTERFACE_INCLUDE_DIRECTORIES "${TELEMANIP_INCLUDE_DIR}"
)

# ============================================================
# 库目标
# ============================================================
add_library(dual_arm_hardware_interface SHARED
  # ---- 基础设施（旧）----
  src/control_api_manager.cpp
  src/hardware_base.cpp

  # ---- 组件抽象层（旧）----
  src/joint_group_base.cpp
  src/gripper_base.cpp

  # ---- 合并双臂（旧，UDP ControlApi）----
  src/dual_arms_hardware_interface.cpp

  # ---- 单臂（旧，保留）----
  src/left_arm_hardware_interface.cpp
  src/right_arm_hardware_interface.cpp
  src/head_hardware_interface.cpp
  src/elevator_hardware_interface.cpp

  # ---- 夹爪（旧，保留）----
  src/left_gripper_hardware_interface.cpp
  src/right_gripper_hardware_interface.cpp

  # ---- 新：telemanip 路径（Zenoh + cuarm_2_2）----
  src/telemanip_session.cpp
  src/dual_arms_telemanip_hardware_interface.cpp
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
    std_msgs::std_msgs
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    lifecycle_msgs::lifecycle_msgs
    Boost::boost
    ament_index_cpp::ament_index_cpp
    dual_arm_v2_2_control_sdk::dual_arm_v2_2_control_sdk
  PRIVATE
    diagnostic_updater::diagnostic_updater
    # telemanip .so —— 通过 IMPORTED target 引入，含头文件路径
    telemanip_capi_cuarm_2_2
    # 静态库链接时需要；动态库也有益无害
    m
    pthread
)

# ============================================================
# rpath —— 让 pluginlib dlopen 我们的 .so 后能找到 telemanip .so
#
#   BUILD_RPATH   : 编译期用
#   INSTALL_RPATH : 安装后用
#
#   $ORIGIN : 与自身 .so 同目录
# ============================================================
set_target_properties(dual_arm_hardware_interface PROPERTIES
  BUILD_RPATH   "${CMAKE_CURRENT_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN"
)

# ============================================================
# 编译选项
# ============================================================
target_compile_options(dual_arm_hardware_interface PRIVATE
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall>
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wextra>
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wpedantic>

  $<$<CONFIG:Debug>:-O0;-g3;-fno-omit-frame-pointer>
  $<$<CONFIG:Release>:-O2;-DNDEBUG>
  $<$<CONFIG:RelWithDebInfo>:-O2;-g;-DNDEBUG>
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

# ---- 安装 telemanip .so 到 lib/ ----
# 由于 INSTALL_RPATH = $ORIGIN，我们的 .so 和它同目录即可被找到。
install(FILES "${TELEMANIP_LIB_PATH}"
  DESTINATION lib)

# ---- 安装 zenoh.json5 到 share/<pkg>/config/ ----
# on_init 里通过 ament_index_cpp::get_package_share_directory()
# 拼出绝对路径。
install(FILES config/zenoh.json5
  DESTINATION share/${PROJECT_NAME}/config)

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
  ament_index_cpp
  dual_arm_v2_2_control_sdk
)

ament_package()