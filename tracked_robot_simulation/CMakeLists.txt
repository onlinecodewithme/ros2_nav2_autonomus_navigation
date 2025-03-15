cmake_minimum_required(VERSION 3.8)
project(tracked_robot_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add obstacle visualizer executable
add_executable(obstacle_visualizer src/obstacle_visualizer.cpp)
ament_target_dependencies(obstacle_visualizer
  rclcpp
  nav_msgs
  visualization_msgs
  geometry_msgs
)

# Add Gazebo obstacle visualizer executable
add_executable(gazebo_obstacle_visualizer src/gazebo_obstacle_visualizer.cpp)
ament_target_dependencies(gazebo_obstacle_visualizer
  rclcpp
  visualization_msgs
  geometry_msgs
  std_msgs
)

# Add Odometry to TF executable
add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_target_dependencies(odom_to_tf
  rclcpp
  nav_msgs
  tf2_msgs
  tf2_ros
  tf2
  geometry_msgs
)

# Install executables
install(TARGETS
  obstacle_visualizer
  gazebo_obstacle_visualizer
  odom_to_tf
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and scripts
install(DIRECTORY
  launch
  config
  worlds
  models
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Make Python scripts executable
install(PROGRAMS
  scripts/simple_teleop.py
  scripts/auto_teleop.py
  scripts/simple_robot_control.py
  scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
