FROM arm64v8/ros:galactic-ros-base

RUN apt update && apt upgrade -y
RUN apt install -y nano vim rsync zip gdb iputils-ping git software-properties-common

WORKDIR /root/ros2_ws/src
SHELL ["/bin/bash", "-c"]
RUN echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc

# External dependencies that are unlikely to change
## Realsense
RUN apt-get install -y ros-$ROS_DISTRO-realsense2-camera
# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
#     add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
#     apt-get install -y librealsense2-utils && \
#     apt-get install -y librealsense2-dev

# RUN git clone https://github.com/IntelRealSense/realsense-ros.git && \
#     cd realsense-ros/ && \
#     git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`

# WORKDIR /root/ros2_ws/
# RUN rosdep install --from-paths src --ignore-src -r -y
# RUN colcon build
# RUN echo 'source /root/ros2_ws/install/setup.bash' >> ~/.bashrc

