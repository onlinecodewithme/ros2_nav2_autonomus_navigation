#!/bin/bash

# Script to start autonomous navigation with Deep Reinforcement Learning
# This uses the TD3 (Twin Delayed Deep Deterministic Policy Gradient) algorithm
# to navigate without a pre-defined map while avoiding obstacles

# Default to simulation mode
use_simulation=true

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --real)
            use_simulation=false
            shift
            ;;
        --sim)
            use_simulation=true
            shift
            ;;
        *)
            # Unknown option
            shift
            ;;
    esac
done

# Source the ROS 2 setup file
source /home/x4/x4_autonomus/install/setup.bash

echo "===== Deep Reinforcement Learning Navigation ====="
echo "This script launches the DRL-based autonomous navigation with the ZED camera."
echo "The robot will navigate to a target location while avoiding obstacles without a pre-defined map."
echo ""

# Display the current mode
if [ "$use_simulation" = true ]; then
    echo "Running in SIMULATION mode"
else
    echo "Running in REAL HARDWARE mode"
fi
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
    
    # Run the DRL navigation with remote RViz enabled
    echo "Starting DRL navigation with remote RViz..."
    if [ "$use_simulation" = true ]; then
        export DISPLAY=:1 && USE_SIMULATION=true python3 /home/x4/x4_autonomus/src/tracked_robot_nav/scripts/drl_navigation/run_drl_navigation.py &
    else
        export DISPLAY=:1 && USE_SIMULATION=false python3 /home/x4/x4_autonomus/src/tracked_robot_nav/scripts/drl_navigation/run_drl_navigation.py &
    fi
    DRL_PID=$!
    
    # Wait a moment for navigation to start
    sleep 5
    
    # Launch RViz separately
    echo "Launching RViz separately..."
    ./$(dirname "$0")/run_rviz_map.sh
    
    # When RViz is closed, ask if we should stop navigation
    echo ""
    echo "RViz was closed. Do you want to stop the DRL navigation? (y/n)"
    read -r stop_nav
    
    if [[ "$stop_nav" =~ ^[Yy]$ ]]; then
        echo "Stopping DRL navigation..."
        kill $DRL_PID
    else
        echo "DRL navigation is still running in the background."
        echo "You can press Ctrl+C to stop it when you're done."
    fi
else
    # Run the DRL navigation with local RViz
    echo "Using local RViz on the Jetson..."
    # Start DRL navigation in the background
    if [ "$use_simulation" = true ]; then
        export DISPLAY=:1 && USE_SIMULATION=true python3 /home/x4/x4_autonomus/src/tracked_robot_nav/scripts/drl_navigation/run_drl_navigation.py &
    else
        export DISPLAY=:1 && USE_SIMULATION=false python3 /home/x4/x4_autonomus/src/tracked_robot_nav/scripts/drl_navigation/run_drl_navigation.py &
    fi
    DRL_PID=$!
    
    # Wait a moment for navigation to start
    sleep 5
    
    # Launch RViz separately
    echo "Launching RViz separately..."
    ./$(dirname "$0")/run_rviz_map.sh
    
    # When RViz is closed, ask if we should stop navigation
    echo ""
    echo "RViz was closed. Do you want to stop the DRL navigation? (y/n)"
    read -r stop_nav
    
    if [[ "$stop_nav" =~ ^[Yy]$ ]]; then
        echo "Stopping DRL navigation..."
        kill $DRL_PID
    else
        echo "DRL navigation is still running in the background."
        echo "You can press Ctrl+C to stop it when you're done."
    fi
fi

echo ""
echo "===== Deep Reinforcement Learning Navigation Started ====="
echo "The robot will navigate to the target location while avoiding obstacles."
echo "The navigation is powered by TD3 deep reinforcement learning algorithm."
