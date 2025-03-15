#!/bin/bash

# Script to test map loading in RViz

echo "===== Testing Map Loading in RViz ====="
echo "This script launches RViz with the map server to test map loading."
echo ""

# Get the directory of this script
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
NAV_DIR=$(dirname "$SCRIPT_DIR")

# Check if map file exists
MAP_FILE="$NAV_DIR/maps/map.yaml"
if [ ! -f "$MAP_FILE" ]; then
    echo "Error: Map file $MAP_FILE does not exist!"
    exit 1
fi

echo "Using map file: $MAP_FILE"

# Launch map server and RViz
echo "Launching map server and RViz..."
export DISPLAY=:1 && ros2 launch tracked_robot_nav test_map_loading.launch.py

echo ""
echo "===== Map Loading Test Complete ====="
