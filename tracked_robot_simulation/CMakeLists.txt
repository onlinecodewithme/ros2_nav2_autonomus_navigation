cmake_minimum_required(VERSION 3.8)
project(tracked_robot_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

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
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
