#!/bin/bash

LOG_FILE="$HOME/nav2.log"
PID_FILE="$HOME/nav2.pid"

setsid bash -lc '
source /opt/ros/humble/setup.bash

source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=40
export LDS_MODEL=LDS-03

ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/ktel/pyqt_ws/src/YEYU_turtlebot3_pyqt_gui/maps/yeyu_map1.yaml
' > "$LOG_FILE" 2>&1 &

echo $! > "$PID_FILE"

echo "STARTED"

