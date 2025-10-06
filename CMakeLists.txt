cmake_minimum_required(VERSION 3.5)
project(robo1_haje)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
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

# ------------------------------------------------------------------------------
# Include directories (since you keep headers inside src/)
# ------------------------------------------------------------------------------
include_directories(src)

# ------------------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------------------

# Example: Drone tree detection node (tree_detector.cpp + tree_detector.hpp)
add_executable(tree_detector src/Detection.cpp)
ament_target_dependencies(tree_detector
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

# You can add others here later, e.g.
# add_executable(movement src/movement.cpp)
# ament_target_dependencies(movement rclcpp geometry_msgs nav2_msgs rclcpp_action std_msgs)

# ------------------------------------------------------------------------------
# Install targets
# ------------------------------------------------------------------------------
install(TARGETS
  tree_detector
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Install other resources (unchanged)
# ------------------------------------------------------------------------------
install(
  DIRECTORY config launch models urdf worlds srv src
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
ament_package()
