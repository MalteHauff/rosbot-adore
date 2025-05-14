#!/bin/bash
set -e

# Source ROS2 Iron
source /opt/ros/iron/setup.bash

# Source workspace, if built
if [ -f "$ROS_WS/install/setup.bash" ]; then
  source "$ROS_WS/install/setup.bash"
fi

# Execute passed command
exec "$@"