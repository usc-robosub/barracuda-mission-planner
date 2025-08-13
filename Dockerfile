# Minimal ROS1 Noetic catkin workspace image
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \
    vim \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*


# Pre-copy the src to allow image builds without bind mounts
COPY . /opt/barracuda-mission-planner

WORKDIR /opt

CMD ["/bin/bash", "/opt/barracuda-mission-planner/entrypoint.sh"]


