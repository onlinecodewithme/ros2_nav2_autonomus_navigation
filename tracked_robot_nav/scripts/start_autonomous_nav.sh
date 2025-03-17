#!/bin/bash

# Script to start autonomous navigation with ZED camera

# Source the ROS 2 setup file
source /home/x4/x4_autonomus/install/setup.bash

echo "===== ZED Camera Autonomous Navigation ====="
echo "This script launches the autonomous navigation stack with the ZED camera."
echo "Make sure your robot is positioned correctly and a map is available."
echo ""

# Ask user if they want to use remote RViz
echo ""
echo "Do you want to use RViz on a remote machine? (y/n)"
read -r use_remote_rviz

# Set the use_remote_rviz parameter based on user input
if [[ "$use_remote_rviz" =~ ^[Yy]$ ]]; then
    # Set up ROS network configuration for remote RViz
    echo "Setting up ROS network for remote visualization..."
    source $(dirname "$0")/setup_ros_network.sh
    
    # Run the autonomous navigation launch file with remote RViz enabled
    echo "Starting navigation with remote RViz..."
    export DISPLAY=:1 && ros2 launch tracked_robot_nav autonomous_nav.launch.py use_remote_rviz:=true
else
    # Run the autonomous navigation launch file without RViz
    echo "Using local RViz on the Jetson..."
    # Start navigation in the background
    export DISPLAY=:1 && ros2 launch tracked_robot_nav autonomous_nav.launch.py use_remote_rviz:=false &
    NAV_PID=$!
    
    # Wait a moment for navigation to start
    sleep 5
    
    # Launch RViz separately
    echo "Launching RViz separately..."
    ./$(dirname "$0")/run_rviz_map.sh
    
    # When RViz is closed, ask if we should stop navigation
    echo ""
    echo "RViz was closed. Do you want to stop the navigation stack? (y/n)"
    read -r stop_nav
    
    if [[ "$stop_nav" =~ ^[Yy]$ ]]; then
        echo "Stopping navigation..."
        kill $NAV_PID
    else
        echo "Navigation is still running in the background."
        echo "You can press Ctrl+C to stop it when you're done."
    fi
fi

echo ""
echo "===== Autonomous Navigation Started ====="
echo "Use RViz to set navigation goals for the robot."
