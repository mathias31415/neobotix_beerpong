#!/bin/sh
echo "Run Container"
xhost + local:root

docker run --name neobotix_mmo500_bringup \
    --privileged \
    -di \
    --group-add=dialout \
    -e DISPLAY=$DISPLAY \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/neobotix/.Xauthority:/home/docker/.Xauthority \
    --net host \
    --restart always \
    --ipc host \
    neobotix_mmo500_bringup/ros:humble

    # -v /home/neobotix/ss24_workspace/src:/home/docker/ros2_ws \
