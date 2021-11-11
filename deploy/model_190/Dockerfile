FROM arm64v8/ros:galactic-ros-base

RUN apt update && apt upgrade -y
RUN apt install -y nano vim rsync zip gdb iputils-ping git software-properties-common \
                   libudev-dev

WORKDIR /root/ros2_ws/src
SHELL ["/bin/bash", "-c"]
RUN echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc

# Realsense
RUN apt-get install -y ros-$ROS_DISTRO-realsense2-camera

# LD06 LiDAR, Sparkfun Artemis IMU
RUN git clone -b ros2 https://github.com/linorobot/ldlidar.git && \ 
    git clone -b ros2 https://github.com/klintan/ros2_razor_imu.git

# Build
WORKDIR /root/ros2_ws
RUN source /opt/ros/galactic/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/galactic/setup.bash && \
    colcon build

# Done
RUN echo 'source /root/ros2_ws/install/setup.bash' >> ~/.bashrc
