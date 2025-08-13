# Barracuda Mission Planner - ROS1 Skeleton

This repository provides a minimal ROS1 (catkin) workspace focused on the `barracuda_mission_planner` package, along with Docker and docker-compose to build and run it in a container.

## Tree

```
├── catkin_ws
│   └── src
│       ├── CMakeLists.txt               # includes catkin toplevel
│       └── barracuda_mission_planner
│           ├── CMakeLists.txt           # minimal catkin package
│           ├── package.xml              # package metadata
│           ├── launch/                  # add launch files here
│           ├── scripts/                 # add Python nodes here
│           └── src/                     # add C++ sources here
├── docker-compose.yml
├── Dockerfile
├── entrypoint.sh
└── README.md
```

## Prerequisites

- Docker and docker-compose (Compose V2).

## Build and Run

1. Build the image:
   - `docker compose build`
2. Start an interactive shell in the container (workspace auto-builds on first run):
   - `docker compose run --rm mission_planner`
3. Force a rebuild of the workspace inside the container if needed:
   - `FORCE_BUILD=1 docker compose run --rm mission_planner`

## Example Node

- Package: `barracuda_mission_planner`
- Node: `mission_routine_node.py` (Python, dummy routine)
- Launch: `roslaunch barracuda_mission_planner mission_routine.launch`

While the launch is running, you can start/stop the routine via topics:

- Start: `rostopic pub /mission/command std_msgs/String "start" -1`
- Stop: `rostopic pub /mission/command std_msgs/String "stop" -1`
- Observe status: `rostopic echo /mission/status`

## Notes

- The container uses ROS Noetic (`ros:noetic-ros-base`).
- The workspace is bind-mounted; your changes on the host are visible in the container.
- Add your nodes, launch files, and other resources under `catkin_ws/src/barracuda_mission_planner` and rebuild.
