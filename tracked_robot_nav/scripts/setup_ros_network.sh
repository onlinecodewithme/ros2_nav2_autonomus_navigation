#!/bin/bash

# Script to set up ROS network configuration on the Jetson
# This ensures communication with the remote RViz2 instance

# Source the ROS 2 setup file
source /home/x4/x4_autonomus/install/setup.bash

# Set ROS_DOMAIN_ID to ensure communication between machines
# Both machines must use the same ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Get the Jetson's IP address
JETSON_IP=$(hostname -I | awk '{print $1}')
echo "Jetson IP: $JETSON_IP"

# Print instructions for the remote machine
echo "============================================================"
echo "ROS network configuration set up on Jetson"
echo "============================================================"
echo "To connect from your remote machine (192.168.1.74):"
echo ""
echo "1. Create directories for RViz configurations and maps on your remote machine:"
echo "   mkdir -p ~/rviz_configs ~/maps"
echo ""
echo "2. Copy the RViz configurations to your remote machine:"
echo "   scp -r /home/x4/x4_autonomus/src/tracked_robot_nav/config/*.rviz user@192.168.1.74:~/rviz_configs/"
echo ""
echo "3. Copy the example map files to your remote machine (optional, for reference):"
echo "   scp -r /home/x4/x4_autonomus/src/example_project/agv_proto/maps/* user@192.168.1.74:~/maps/"
echo ""
echo "4. Copy the run_rviz_remote.sh script to your remote machine:"
echo "   scp /home/x4/x4_autonomus/src/tracked_robot_nav/scripts/run_rviz_remote.sh user@192.168.1.74:~/"
echo ""
echo "5. On your remote machine, edit the run_rviz_remote.sh script to:"
echo "   - Set the Jetson IP address to: $JETSON_IP"
echo ""
echo "6. On your remote machine, run:"
echo "   export ROS_DOMAIN_ID=42"
echo "   ./run_rviz_remote.sh [slam|navigation]"
echo "============================================================"
echo ""
echo "This terminal is now configured for ROS communication with the remote machine."
echo "Run your ROS commands from this terminal."
