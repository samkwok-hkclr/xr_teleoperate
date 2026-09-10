cmake_minimum_required(VERSION 3.8)
project(xr_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Message packages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(xr_teleop_msgs REQUIRED)

# TF2 packages
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# MoveIt packages
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_toolbox REQUIRED)

find_package(xr_teleop_msgs REQUIRED)

include_directories(include)

# -----------------------------------------------------------
# Pose Tracker (Lifecycle Composable Node Component)
# -----------------------------------------------------------
add_library(pose_tracker_component SHARED src/pose_tracker.cpp)
ament_target_dependencies(pose_tracker_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  tf2_eigen
  tf2_geometry_msgs
  moveit_core
  moveit_servo
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  control_msgs
  control_toolbox
)

# Register the component
rclcpp_components_register_node(pose_tracker_component
  PLUGIN "PoseTrackerWrapper"
  EXECUTABLE pose_tracker
)

# -----------------------------------------------------------
# XR Teleop (Lifecycle Composable Node Component)
# -----------------------------------------------------------
add_library(xr_teleop_component SHARED src/xr_teleop.cpp)
ament_target_dependencies(xr_teleop_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  xr_teleop_msgs
)

rclcpp_components_register_node(xr_teleop_component
  PLUGIN "XrTeleop"
  EXECUTABLE xr_teleop
)

# -----------------------------------------------------------
# Lifecycle Manager Executable
# -----------------------------------------------------------
add_executable(lifecycle_manager src/lifecycle_manager.cpp)
ament_target_dependencies(lifecycle_manager
  rclcpp
  lifecycle_msgs
  std_srvs
)

# -----------------------------------------------------------
# Install Rules
# -----------------------------------------------------------
# Install component shared libraries
install(TARGETS 
  pose_tracker_component
  xr_teleop_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS
  lifecycle_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (important if you have custom headers like arm_types.hpp)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
