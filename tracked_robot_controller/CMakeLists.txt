cmake_minimum_required(VERSION 3.8)
project(tracked_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# cmd_vel to tracks node
add_executable(cmd_vel_to_tracks src/cmd_vel_to_tracks.cpp)
ament_target_dependencies(cmd_vel_to_tracks
  rclcpp
  geometry_msgs
  std_msgs
)

# track odometry node
add_executable(track_odometry src/track_odometry.cpp)
ament_target_dependencies(track_odometry
  rclcpp
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
)

include_directories(
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

# Install executables
install(TARGETS
  cmd_vel_to_tracks
  track_odometry
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()