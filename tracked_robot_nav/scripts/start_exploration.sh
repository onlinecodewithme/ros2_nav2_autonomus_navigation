#!/bin/bash

# Script to start autonomous exploration with ZED camera

echo "===== ZED Camera Autonomous Exploration ====="
echo "This script launches the autonomous exploration stack with the ZED camera."
echo "The robot will automatically explore and map the environment."
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
    
    # Run the exploration launch file with remote RViz enabled
    export DISPLAY=:1 && ros2 launch tracked_robot_nav exploration.launch.py use_remote_rviz:=true
else
    # Run the exploration launch file with local RViz
    echo "Using local RViz on the Jetson..."
    export DISPLAY=:1 && ros2 launch tracked_robot_nav exploration.launch.py use_remote_rviz:=false
fi

echo ""
echo "===== Autonomous Exploration Started ====="
echo "The robot will automatically explore the environment."
echo "After exploration is complete, save your map using:"
echo "ros2 run nav2_map_server map_saver_cli -f ~/x4_autonomus/src/tracked_robot_nav/maps/map"
