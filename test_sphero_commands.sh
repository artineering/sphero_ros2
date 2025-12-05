#!/bin/bash
# Test script to send multiple ROS2 commands to Sphero using command line

# Configuration
SPHERO_NAME="${1:-SB-1234}"
TOPIC_PREFIX="sphero/${SPHERO_NAME}"

echo "==============================================="
echo "Testing Sphero Commands for: $SPHERO_NAME"
echo "==============================================="


# Function to send roll command
send_roll() {
    local heading=$1
    local speed=$2
    local duration=$3
    echo "Roll: heading=${heading}° speed=$speed duration=${duration}s"
    ros2 topic pub --once "${TOPIC_PREFIX}/roll" std_msgs/msg/String \
        "{data: '{\"heading\": $heading, \"speed\": $speed, \"duration\": $duration}'}"
}

