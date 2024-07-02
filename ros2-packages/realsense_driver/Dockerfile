##############################################################################
##                           1. stage: Base Image                           ##
##############################################################################
ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop as base

# Configure DDS
COPY dds_profile.xml /opt/misc/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/misc/dds_profile.xml

# Create user
ARG USER=docker_realsense
ARG UID=1000
ARG GID=1000
ENV USER=$USER
RUN groupadd -g $GID $USER \
    && useradd -m -u $UID -g $GID --shell $(which bash) $USER


# Setup workpace
USER $USER
RUN mkdir -p /home/$USER/ros2_ws/src
WORKDIR /home/$USER/ros2_ws


##############################################################################
##                   2. stage: install realsense content with apt           ##
##############################################################################
FROM base as realsense_ros2

# Install realsense packages
USER root
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-realsense2-description\
    #####fuma1017
    ros-$ROS_DISTRO-tf-transformations
USER $USER

# Install Python
USER root
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    python3-numpy \
    libboost-python-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U pip setuptools
RUN pip3 install opencv-python
###fuma1017
RUN pip3 install opencv-contrib-python transforms3d
USER $USER

# COPY source flder into container
COPY ./src /home/$USER/ros2_ws/

# Build the workspace package (application)
RUN cd /home/$USER/ros2_ws && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

# Add built package to entrypoint by calling install/setup.bash
USER root
RUN sed -i 's|exec "\$@"|source "/home/'"${USER}"'/ros2_ws/install/setup.bash"\n&|' /ros_entrypoint.sh
USER $USER


# autostart bringup with rviz
CMD ["ros2", "launch", "neobotix_realsense415", "realsense_bringup.launch.py"]