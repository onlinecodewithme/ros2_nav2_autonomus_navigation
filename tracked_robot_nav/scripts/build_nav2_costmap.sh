#!/bin/bash

# Script to build Nav2 packages for costmap visualization

echo "===== Building Nav2 Packages for Costmap Visualization ====="
echo "This script will build only the necessary Nav2 packages for costmap visualization."
echo ""

# Navigate to the workspace root
cd /home/x4/ros2_ws

# Build only the necessary packages
colcon build --packages-select nav2_costmap_2d nav2_map_server nav2_lifecycle_manager

echo ""
echo "===== Build Completed ====="
echo "You can now run the costmap test with: ./tracked_robot_nav/scripts/test_costmap.sh"
