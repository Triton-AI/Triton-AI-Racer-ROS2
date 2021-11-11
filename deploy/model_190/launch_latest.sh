#! /bin/bash
docker pull haoru233/triton-ai-racer-ros2:arm64-galactic-190
docker run -it --rm --update --name tai-racer-ros2-190 \
    --privileged \
    --net=host \
    haoru233/triton-ai-racer-ros2:arm64-galactic-190