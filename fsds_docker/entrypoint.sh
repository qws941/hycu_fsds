#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

export PYTHONPATH=/root/catkin_ws/src/fsds_scripts:${PYTHONPATH}

exec "$@"
