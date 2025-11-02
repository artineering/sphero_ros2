#!/bin/bash
# Test script to manually test the sphero_controller_node with a parameter

echo "=========================================="
echo "  Testing Sphero Controller Node"
echo "=========================================="
echo ""

# Source workspace
source /home/svaghela/ros2_ws_2/install/setup.bash

# Get Sphero name from user
read -p "Enter Sphero name (e.g., SB-3660): " SPHERO_NAME

if [ -z "$SPHERO_NAME" ]; then
    echo "Error: Sphero name cannot be empty"
    exit 1
fi

echo ""
echo "Testing controller with Sphero: $SPHERO_NAME"
echo ""
echo "Command that will be executed:"
echo "  ros2 run sphero_package sphero_controller_node --ros-args -p toy_name:=$SPHERO_NAME"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run the controller node
ros2 run sphero_package sphero_controller_node --ros-args -p toy_name:=$SPHERO_NAME
