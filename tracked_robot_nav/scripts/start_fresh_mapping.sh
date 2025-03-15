#!/bin/bash

# Script to start fresh mapping with ZED camera
# This script uses the updated configuration with extended z-axis bounds and simplified launch

echo "===== ZED Camera Fresh Mapping ====="
echo "Configuration changes made:"
echo " - Extended z-axis bounds to include sensor at z=-1.16"
echo " - Changed origin_z from 0.0 to -1.5"
echo " - Increased z_voxels from 10 to 35"
echo " - Added full Nav2 navigation stack for costmap visualization"
echo " - Configured lifecycle manager to manage all navigation nodes"
echo " - Added proper costmap visualization in RViz"
echo ""
echo "Starting fresh mapping with ZED camera..."
echo "Make sure your robot is positioned correctly before mapping."
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
    
    # Run the mapping launch file with remote RViz enabled
    export DISPLAY=:1 && ros2 launch tracked_robot_nav mapping_zed.launch.py use_remote_rviz:=true
else
    # Run the mapping launch file with local RViz
    echo "Using local RViz on the Jetson..."
    export DISPLAY=:1 && ros2 launch tracked_robot_nav mapping_zed.launch.py use_remote_rviz:=false
fi

echo ""
echo "===== After mapping is complete ====="
echo "Save your new map using:"
echo "ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/tracked_robot_nav/maps/map"
