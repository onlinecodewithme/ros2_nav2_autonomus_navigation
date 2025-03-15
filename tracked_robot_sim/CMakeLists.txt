cmake_minimum_required(VERSION 3.8)
project(tracked_robot_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Find optional dependencies with proper error handling
find_package(urdf QUIET)
find_package(xacro QUIET)

# Look for ros_gz packages (Gazebo Garden integration with ROS 2)
find_package(ros_gz QUIET)
find_package(ros_gz_sim QUIET)
find_package(ros_gz_bridge QUIET)

# Set conditional flags
if(ros_gz_sim_FOUND)
  message(STATUS "Found ros_gz_sim: using Gazebo features")
  add_definitions(-DUSE_GZ_SIM=1)
else()
  message(STATUS "ros_gz_sim NOT found: disabling Gazebo features")
  add_definitions(-DUSE_GZ_SIM=0)
endif()

if(urdf_FOUND)
  message(STATUS "Found urdf")
else()
  message(STATUS "urdf NOT found: some features may be disabled")
endif()

# Install Python modules (if they exist)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME})
endif()

# Create scripts directory if it doesn't exist
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts)

# Install Python executables (if they exist)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/track_controller_node.py)
  install(PROGRAMS
    scripts/track_controller_node.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install directories conditionally
# Launch files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Config files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# URDF files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/urdf)
  install(DIRECTORY
    urdf
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# World files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/worlds)
  install(DIRECTORY
    worlds
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Model files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/models)
  install(DIRECTORY
    models
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# RViz configuration
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY
    rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()