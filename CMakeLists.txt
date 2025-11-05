cmake_minimum_required(VERSION 3.5)
project(robo1_haje)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros_ign_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TinyXML2 REQUIRED)

# Required Package for Python Integration
find_package(ament_cmake_python REQUIRED)

# Install the Python package (so robo1_haje/ becomes importable)
ament_python_install_package(${PROJECT_NAME})

# Install your Python executable script into ROS 2's lib directory
install(PROGRAMS
  robo1_haje/movement_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Include directories (since you keep headers inside src/)
include_directories(src)

# Executables
add_executable(tree_detector src/Detection.cpp)
ament_target_dependencies(tree_detector
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  cv_bridge
  image_transport
  OpenCV
  TinyXML2
  std_msgs
)

# Install targets
install(TARGETS
  tree_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install other resources
install(
  DIRECTORY config launch models urdf worlds src
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
