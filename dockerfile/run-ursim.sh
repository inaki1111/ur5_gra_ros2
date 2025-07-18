#!/usr/bin/env bash

# get script directory as absolute path
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# get repository directory as absolute path
REPOSITORY_DIR="$( cd "$( dirname "${SCRIPT_DIR}" )" &> /dev/null && pwd )/colcon_ws"

xhost +

docker run \
--rm \
--tty \
--interactive \
--privileged \
--publish 5900:5900 \
--publish 6080:6080 \
--net ursim_net  \
--ip 192.168.56.101 \
--env DISPLAY=$DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--volume $(SCRIPT_DIR)/../ursim/urcaps:/urcaps \
--volume $(SCRIPT_DIR)/../ursim/programs:/ursim/urcaps \
--name ursim universalrobots/ursim_e-series \
ros2-urgra:latestursim universalrobots/ursim_e-series \
bash

#docker run --rm -it -p 5900:5900 -p 6080:6080 --net ursim_net --ip 192.168.56.101 /home/pissard/Dev/Thot/universal_robots/ursim/urcaps:/urcaps -v /home/pissard/Dev/Thot/universal_robots/ursim/programs:/ursim/programs --name ursim universalrobots/ursim_e-series