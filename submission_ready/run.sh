#!/bin/bash
# Usage: ./run.sh <WINDOWS_IP>
# Example: ./run.sh 192.168.1.100

IP=${1:-$FSDS_HOST_IP}
[ -z "$IP" ] && echo "Usage: ./run.sh <WINDOWS_IP>" && exit 1

docker build -t fsds .
docker run -it --rm --net=host -e FSDS_HOST_IP=$IP -v $(pwd)/scripts:/scripts fsds bash -c "
source /opt/ros/noetic/setup.bash
source /root/Formula-Student-Driverless-Simulator/ros/devel/setup.bash
roslaunch fsds_ros_bridge fsds_ros_bridge.launch host:=$IP &
sleep 5
python3 /scripts/competition_driver.py
"
