cmake_minimum_required(VERSION 3.8)
project(navigation_demo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_custom_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

# Add include directories
include_directories(include)

# ============================================
# Executables
# ============================================
add_executable(set_goal_server src/set_goal_server.cpp)
ament_target_dependencies(set_goal_server rclcpp robot_custom_interfaces)

add_executable(set_goal_client src/set_goal_client.cpp)
ament_target_dependencies(set_goal_client rclcpp robot_custom_interfaces)

add_executable(navigate_to_pose_client src/navigate_to_pose_client.cpp)
ament_target_dependencies(navigate_to_pose_client 
  rclcpp 
  robot_custom_interfaces
  rclcpp_action)
  add_executable(navigate_to_pose_server src/navigate_to_pose_server.cpp)
  ament_target_dependencies(navigate_to_pose_server 
  rclcpp 
  robot_custom_interfaces
  rclcpp_action)
  
  
# ============================================
# Install
# ============================================
install(TARGETS
  set_goal_server
  set_goal_client
  navigate_to_pose_server
  navigate_to_pose_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
