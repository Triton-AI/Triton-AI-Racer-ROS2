#! /bin/bash
docker run -it --rm --update --name tai-racer-ros2-190 \
    --privileged \
    --net=host \
    --mount type=bind,source="$(pwd)"/../../src,target=/root/ros2_ws/src
    haoru233/triton-ai-racer-ros2:arm64-galactic-190