#!/bin/bash
set -e

# Source ROS 2 base
source /opt/ros/jazzy/setup.bash

# Source the workspace overlay if it exists
if [ -f "/ws/install/setup.bash" ]; then
    source "/ws/install/setup.bash"
fi

# Execute the passed command (or bash if none)
if [ $# -eq 0 ]; then
    exec bash
else
    exec "$@"
fi
