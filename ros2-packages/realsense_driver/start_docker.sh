#!/bin/sh
echo "Run Container"
xhost + local:root

#    -v /dev:/dev \
docker run \
    --name realsense_driver \
    --privileged \
    -it\
    -e DISPLAY=$DISPLAY \
    --env-file .env\
    -v $PWD/src:/home/docker_realsense/ros2_ws/src \
    --net host \
    --rm realsense_driver/ros:humble