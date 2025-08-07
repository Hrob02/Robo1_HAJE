cmake_minimum_required(VERSION 3.5)
project(robo1_haje)

# Install all launch files under launch/ into share/Robo1_HAJE/launch/
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# These packages may be subject to change
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Service files
## rosidl_generate_interfaces(${PROJECT_NAME}
#  "srv/AddGoal.srv"
#  "srv/ExecuteGoals.srv"
#)

add_executable(Movement src/Movement.cpp)
ament_target_dependencies(Movement rclcpp nav2_msgs rclcpp_action geometry_msgs std_msgs)

# Link generated services
## rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
## target_link_libraries(movementlogic ${cpp_typesupport_target})

install(TARGETS
  Movement
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
