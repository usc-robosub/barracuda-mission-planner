set -euo pipefail

source /opt/ros/noetic/setup.bash

cd /opt/barracuda-mission-planner/catkin_ws
catkin_make

exec /bin/bash
