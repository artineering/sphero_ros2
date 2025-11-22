#!/bin/bash
# Soccer Game Controller Startup Script
# This script provides a clean interface by starting components in the right order

# Source ROS2 workspace
source /home/svaghela/ros2_ws_2/install/setup.bash

# Start multi-robot webserver in background
echo "Starting multi-robot webserver..."
ros2 run multirobot_webserver multirobot_webapp > /dev/null 2>&1 &
WEBSERVER_PID=$!

# Wait for webserver to initialize
sleep 2

# Clear screen for clean interface
clear

# Start soccer game controller (interactive)
ros2 run soccer_game_controller soccer_game_controller

# Cleanup: Kill webserver when controller exits
kill $WEBSERVER_PID 2>/dev/null
