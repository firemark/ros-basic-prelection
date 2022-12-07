#!/bin/bash
./_run.sh \
    --gpus=all \
    --env=NVIDIA_VISIBLE_DEVICES=all \
    --env=NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    ros_prelection "$@"
