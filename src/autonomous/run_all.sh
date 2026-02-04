#!/bin/bash
set -e

# Source ROS setup
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

echo "=== FSDS Competition Driver ==="

# Load parameters from config file
echo "Loading parameters..."
rosparam load /root/catkin_ws/src/autonomous/config/params.yaml /competition_driver

echo "Starting Lap Timer..."
python3 /root/catkin_ws/src/autonomous/modules/utils/lap_timer.py &
LAP_PID=$!

echo "Starting Driver..."
# Redirect logs to file AND stdout for docker logs
python3 /root/catkin_ws/src/autonomous/driver/competition_driver.py _target_laps:=${TARGET_LAPS:-3} 2>&1 | tee /root/driver.log &
DRIVER_PID=$!

# Wait for any process to exit
wait -n $LAP_PID $DRIVER_PID
