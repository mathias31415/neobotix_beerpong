##############################################################################
##                                 Base Image                               ##
##############################################################################
FROM hkairas/ros-kuka-eki:humble

##############################################################################
##                                  User                                    ##
##############################################################################
ARG USER=robot
ARG PASSWORD=robot
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=53
ENV UID=${UID}
ENV GID=${GID}
ENV USER=${USER}
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp

RUN echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc

RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir flask

USER $USER 
RUN rosdep update



RUN mkdir -p /home/$USER/ros_ws/src

WORKDIR /home/$USER/ros_ws
RUN colcon build
#RUN install/setup.bash
CMD /bin/bash

#WORKDIR /home/robot/ros_ws/src/

#ENTRYPOINT ["python3", "pkg_website_beerpong/pkg_website_beerpong/app.py"]


