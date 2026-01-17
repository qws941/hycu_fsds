#!/bin/bash
set -e

# Source ROS
source /opt/ros/noetic/setup.bash

# Source FSDS workspace if exists
if [ -f /root/Formula-Student-Driverless-Simulator/ros/devel/setup.bash ]; then
    source /root/Formula-Student-Driverless-Simulator/ros/devel/setup.bash
fi

# Source custom workspace if exists
if [ -f /root/catkin_ws/devel/setup.bash ]; then
    source /root/catkin_ws/devel/setup.bash
fi

exec "$@"
