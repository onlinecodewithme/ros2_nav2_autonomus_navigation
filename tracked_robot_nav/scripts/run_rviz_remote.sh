#!/bin/bash

# Script to run RViz2 on a remote machine (192.168.1.74)
# This script should be copied to the remote machine and executed there

# Set ROS_DOMAIN_ID to ensure communication between machines
# Both machines must use the same ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42

# Set the IP address of the Jetson (replace with the actual IP of your Jetson)
JETSON_IP="192.168.1.X"  # REPLACE THIS with the actual IP shown by the setup_ros_network.sh script
echo "Using Jetson IP: $JETSON_IP"

# Set ROS environment variables to communicate with the Jetson
export ROS_LOCALHOST_ONLY=0

# Choose which RViz configuration to use based on the argument
CONFIG_TYPE=${1:-"slam"}  # Default to slam if no argument provided

if [ "$CONFIG_TYPE" == "navigation" ]; then
    RVIZ_CONFIG="navigation.rviz"
    echo "Using navigation configuration"
elif [ "$CONFIG_TYPE" == "slam" ]; then
    RVIZ_CONFIG="slam.rviz"
    echo "Using SLAM configuration"
else
    echo "Unknown configuration type: $CONFIG_TYPE"
    echo "Using default SLAM configuration"
    RVIZ_CONFIG="slam.rviz"
fi

# Run RViz2 with the selected configuration
echo "Starting RViz2 with $RVIZ_CONFIG configuration..."
ros2 run rviz2 rviz2 -d ~/rviz_configs/$RVIZ_CONFIG

# Note: You need to copy the RViz configuration files from the Jetson to your local machine:
# scp user@jetson_ip:/home/x4/ros2_ws/src/tracked_robot_nav/config/*.rviz ~/rviz_configs/
#
# If you want to visualize the map from the example project, make sure the map server is running
# on the Jetson. The map server is started automatically when you run the start_fresh_mapping.sh,
# start_autonomous_nav.sh, or start_exploration.sh scripts.
#
# The map is published on the /map topic, which is already configured in the RViz configuration.

echo "RViz2 closed."
