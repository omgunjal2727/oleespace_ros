#!/bin/bash

echo "🔍 Launching YDLidar TG50..."

# Source workspace
source ~/ros2_ws/install/setup.bash

# Set permissions
sudo chmod 666 /dev/ttyUSB0

# Launch with RViz
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py
