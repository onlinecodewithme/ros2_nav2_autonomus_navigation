#!/bin/bash

# Script to test costmap visualization with ZED camera

echo "===== ZED Camera Costmap Test ====="
echo "This script launches the costmap test with the ZED camera."
echo "It will visualize the costmap in RViz to verify proper configuration."
echo ""

# Launch the costmap test
export DISPLAY=:1 && ros2 launch tracked_robot_nav costmap_test.launch.py

echo ""
echo "===== Costmap Test Completed ====="
