#!/usr/bin/env bash

# ROS2 version
ROSVERSION=jazzy

# Directorio actual y volumen del usuario
CURRENT_DIR=$(realpath .)
USERVOLUME=${CURRENT_DIR}/../example

# Permitir conexiones X11 desde el contenedor
xhost +local:root

docker run \
--rm \
--tty \
--interactive \
--gpus all \
--runtime=nvidia \
--privileged \
--network host \
--ipc=host  \
--pid=host  \
--env DISPLAY=$DISPLAY \
--env NVIDIA_DRIVER_CAPABILITIES=all \
--env NVIDIA_VISIBLE_DEVICES=all \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
--volume /dev:/dev \
--volume $USERVOLUME:/root/workspace/example:rw \
--device-cgroup-rule='c 81:* rmw' \
--name ros2-ur5-$ROSVERSION \
ros2-ur5-$ROSVERSION:latest \
bash
