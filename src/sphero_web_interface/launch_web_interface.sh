#!/bin/bash
# Quick launch script for Sphero Web Interface

echo "=========================================="
echo "  Sphero Web Interface Launcher"
echo "=========================================="
echo ""

# Source ROS2 workspace
echo "Sourcing ROS2 workspace..."
source /home/svaghela/ros2_ws_2/install/setup.bash

echo ""
echo "Starting Sphero Web Interface..."
echo "The web interface will be available at: http://localhost:5000"
echo ""
echo "Press Ctrl+C to stop the server"
echo ""

# Run the web server
ros2 run sphero_web_interface web_server
