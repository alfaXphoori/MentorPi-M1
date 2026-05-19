#!/bin/bash
# Script to start FSD on MentorPi M1

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace (assuming standard location)
# Adjust the path if your workspace is elsewhere
WORKSPACE_PATH="/home/ubuntu/ros2_ws"
if [ -d "$WORKSPACE_PATH" ]; then
    source "$WORKSPACE_PATH/install/setup.bash"
fi

# Launch the joint FSD file
ros2 launch mycar fsd.launch.py
