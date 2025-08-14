#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

cd /opt/barracuda-mission-planner/catkin_ws
catkin_make

source /opt/barracuda-mission-planner/catkin_ws/devel/setup.bash

exec roslaunch barracuda_mission_planner pipeline_test.launch
