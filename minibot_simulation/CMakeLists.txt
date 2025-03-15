cmake_minimum_required(VERSION 3.8)
project(minibot_simulation)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# Install directories
install(DIRECTORY
  launch
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()