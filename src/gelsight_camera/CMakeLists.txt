cmake_minimum_required(VERSION 3.8)
project(gelsight_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SlideAlongCable.action"
  "srv/StartSlideAlongCable.srv"
  "srv/StopSlideAlongCable.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Add executables
add_executable(gelsight_camera_node src/gelsight_camera_node.cpp)
add_executable(tactile_sensor_node src/tactile_sensor_node.cpp)
add_executable(admittance_controller src/admittance_controller.cpp)

ament_target_dependencies(gelsight_camera_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV
)

ament_target_dependencies(tactile_sensor_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs
  std_msgs
  cv_bridge 
  OpenCV
)

ament_target_dependencies(admittance_controller 
  rclcpp 
  geometry_msgs
  std_msgs
  std_srvs
)

target_link_libraries(admittance_controller Eigen3::Eigen)

# Install targets
install(TARGETS
  gelsight_camera_node
  tactile_sensor_node
  admittance_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  src/slide_along_cable_primitive.py
  src/slide_cable_client.py
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