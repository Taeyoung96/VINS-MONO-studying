#!/bin/bash
 
set -e

# Ros build
source "/opt/ros/melodic/setup.bash"

echo "==============VINS-MONO Docker Env Ready================"

cd /home/catkin_ws

exec "$@"
