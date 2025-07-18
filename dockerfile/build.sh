#!/usr/bin/env bash

# ROS2 version
ROSVERSION=jazzy

docker build \
  --no-cache \
  --pull \
  -f Dockerfile.${ROSVERSION} \
  -t ros2-ur5-${ROSVERSION}:latest \
  .


#docker build \
#  --no-cache \
#  --pull \
#  -f Dockerfile.${ROSVERSION} \
#  -t ros2-ur5-${ROSVERSION}:latest \
#  .
