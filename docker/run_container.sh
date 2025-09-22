#!/bin/bash
xhost +local:root
IMAGE="omnilrs-navigation:v1.0"
NAME="omnilrs-navigation-container"

# Host path where your bags live:
HOST_BAGS="/home/yhofmann/git/grand_tour_dataset"


DOCKER_RUN_CMD="docker run -it --rm --privileged \
                --gpus all \
                --runtime=nvidia \
                -e NVIDIA_VISIBLE_DEVICES=all \
                -e NVIDIA_DRIVER_CAPABILITIES=all \
                -e XDG_RUNTIME_DIR=/tmp/runtime-docker \
                -e "ACCEPT_EULA=Y" \
                -e "PRIVACY_CONSENT=Y" \
                -e "DISPLAY=$DISPLAY" \
                -v $HOME/.Xauthority:/root/.Xauthority \
                -v $PWD/docker:/docker \
                -v $PWD/humble_ws:/ros2_ws \
                -v "$HOST_BAGS":"$HOST_BAGS":rw \
                --env DISPLAY=$DISPLAY
                --volume /tmp/.X11-unix:/tmp/.X11-unix \
                --network=host \
                --ipc=host \
                --name $NAME \
                $IMAGE"
#-v /dev/:/dev/ \
# Create runtime directory
mkdir -p /tmp/runtime-docker

exec ${DOCKER_RUN_CMD[*]}
