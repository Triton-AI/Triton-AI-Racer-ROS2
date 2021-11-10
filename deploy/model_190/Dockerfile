FROM arm64v8/ros:galactic-ros-base

RUN apt update && apt upgrade -y
RUN apt install -y nano vim rsync zip gdb iputils-ping git software-properties-common

WORKDIR /root/ros2_ws/src
SHELL ["/bin/bash", "-c"]
RUN echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc

# Realsense
RUN apt-get install -y ros-$ROS_DISTRO-realsense2-camera
