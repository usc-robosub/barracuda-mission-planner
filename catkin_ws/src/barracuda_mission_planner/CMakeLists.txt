cmake_minimum_required(VERSION 3.0.2)
project(barracuda_mission_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  barracuda_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python nodes
catkin_install_python(
  PROGRAMS
    scripts/mission_routine_node.py
    scripts/object_to_waypoints_node.py
    scripts/mission_command_sequencer_node.py
    scripts/test_pipeline_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
