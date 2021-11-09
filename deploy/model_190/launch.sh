#! /bin/bash
docker run -it --rm --name tai-racer-ros2-190 \
    --privileged \
    --net=host \
    haoru233/triton-ai-racer-ros2:arm64-galactic-190