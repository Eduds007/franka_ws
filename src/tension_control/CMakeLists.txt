cmake_minimum_required(VERSION 3.8)
project(tension_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SlideAlongCable.action"
  DEPENDENCIES geometry_msgs std_msgs
)

# Add executables (only for files that exist)
add_executable(admittance_controller src/admittance_controller.cpp)
add_executable(move_homing src/move_homing.cpp)

ament_target_dependencies(admittance_controller 
  rclcpp 
  geometry_msgs
  std_msgs
  std_srvs
)

ament_target_dependencies(move_homing 
  rclcpp 
  rclcpp_action
  geometry_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  sensor_msgs
)

target_link_libraries(admittance_controller Eigen3::Eigen)

# Install targets
install(TARGETS
  admittance_controller
  move_homing
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  src/slide_along_cable_primitive.py
  src/slide_cable_client.py
  scripts/gripper_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()