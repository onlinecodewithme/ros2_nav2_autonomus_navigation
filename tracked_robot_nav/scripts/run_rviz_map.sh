#!/bin/bash

# Script to run RViz with navigation configuration

echo "===== Running RViz with Navigation Configuration ====="
echo "This script launches RViz with a configuration for navigation visualization."
echo ""

# Get the package directory
NAV_DIR=$(ros2 pkg prefix tracked_robot_nav)/share/tracked_robot_nav

# Check if the navigation configuration exists
if [ -f "$NAV_DIR/config/navigation.rviz" ]; then
    echo "Using navigation configuration: $NAV_DIR/config/navigation.rviz"
    CONFIG_FILE="$NAV_DIR/config/navigation.rviz"
elif [ -f "$NAV_DIR/config/simple_map.rviz" ]; then
    echo "Using simple map configuration: $NAV_DIR/config/simple_map.rviz"
    CONFIG_FILE="$NAV_DIR/config/simple_map.rviz"
else
    echo "No RViz configuration found. Using default."
    CONFIG_FILE=""
fi

# Launch RViz with the configuration
echo "Launching RViz..."
export DISPLAY=:1 && ros2 run rviz2 rviz2 -d "$CONFIG_FILE"

echo ""
echo "===== RViz Closed ====="
