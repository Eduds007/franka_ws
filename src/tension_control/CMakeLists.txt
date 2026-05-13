cmake_minimum_required(VERSION 3.8)
project(tension_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# ros2_control / Franka plugin dependencies
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(franka_hardware REQUIRED)
find_package(franka_msgs REQUIRED)

# ── Multi-priority controller (ros2_control plugin) ─────────────────────────
add_library(multi_priority_controller SHARED
  src/multi_priority_controller.cpp
)
target_include_directories(multi_priority_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(multi_priority_controller
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  geometry_msgs
  franka_semantic_components
  franka_hardware
  franka_msgs
)
target_link_libraries(multi_priority_controller Eigen3::Eigen)
pluginlib_export_plugin_description_file(controller_interface tension_control_plugins.xml)

# ── Standalone executables ───────────────────────────────────────────────────
add_executable(admittance_controller src/admittance_controller.cpp)
ament_target_dependencies(admittance_controller
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
)
target_link_libraries(admittance_controller Eigen3::Eigen)

add_executable(move_homing src/move_homing.cpp)
ament_target_dependencies(move_homing
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  sensor_msgs
)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS
  multi_priority_controller
  admittance_controller
  move_homing
  EXPORT export_tension_control
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/gripper_control.py
  scripts/cable_grasp_orchestrator.py
  scripts/circle_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_tension_control HAS_LIBRARY_TARGET)
ament_package()