#! /bin/bash
docker pull haoru233/triton-ai-racer-ros2:arm64-galactic-190
docker run -it --rm --name tai-racer-ros2-190 \
    --privileged \
    --net=host \
    --mount type=bind,source=/home/jetson/haoru/dev,target=/root/ros2_ws/src \
    haoru233/triton-ai-racer-ros2:arm64-galactic-190
