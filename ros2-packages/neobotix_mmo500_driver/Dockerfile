##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-* \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-topic-tools \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-ur \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y nano

##############################################################################
##            Create User changed DOMAIN_ID from 1 to 53                    ##
##############################################################################
ARG USER=docker
ARG PASSWORD=neobotix
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=53
ENV ROS_DOMAIN_ID=${DOMAIN_ID}
ENV UID=${UID}
ENV GID=${GID}
ENV USER=${USER}
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc

COPY dds_profile.xml /home/$USER
RUN chown $USER:$USER /home/$USER/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/dds_profile.xml

USER $USER 
RUN mkdir -p /home/$USER/ros2_ws/src

##############################################################################
##                                 User Dependecies                         ##
##############################################################################

WORKDIR /home/$USER/ros2_ws/src
#RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_mpo_500-2.git --> use mounted repo, because we want to change the robot_description
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_nav2_bringup.git
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_local_planner2.git
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_localization2.git
RUN git clone --branch master          https://github.com/neobotix/neo_common2
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_relayboard_v2-2
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_kinematics_mecanum2.git
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_sick_s300-2
RUN git clone --branch $ROS_DISTRO     https://github.com/neobotix/neo_teleop2
RUN git clone --branch master          https://github.com/neobotix/neo_msgs2
RUN git clone --branch master          https://github.com/neobotix/neo_srvs2

# copy custom packages with full robot description to the workspace src folder
COPY src/neo_mpo_500-2-humble /home/$USER/ros2_ws/src/neo_mpo_500-2
COPY src/description_beerpong /home/$USER/ros2_ws/src/description_beerpong

##############################################################################
##                                 Build ROS and run                        ##
##############################################################################
WORKDIR /home/$USER/ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install
RUN echo "source /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc
RUN echo "export LC_NUMERIC="en_US.UTF-8" " >> ~/.bashrc

RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/ros2_ws/install/setup.bash"' \
    /ros_entrypoint.sh

CMD ["ros2", "launch", "neo_mpo_500-2", "bringup.launch.py"] 

# CMD /bin/bash
