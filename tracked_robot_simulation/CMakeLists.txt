cmake_minimum_required(VERSION 3.8)
project(tracked_robot_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  worlds
  config
  urdf
  maps
  tracked_robot_simulation
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  tracked_robot_simulation/autonomous_navigation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
