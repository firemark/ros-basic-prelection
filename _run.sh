#!/bin/bash
XSOCK=/tmp/.X11-unix
XAUTH=${XAUTHORITY:-$HOME/.Xauthority}
docker run \
    --interactive \
    --tty \
    --rm \
    --volume=$XAUTH:/tmp/.docker.xauth:ro \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=/dev:/dev:rw \
    --volume=$(pwd)/config:/config:rw \
    --privileged \
    --env=DISPLAY=$DISPLAY \
    --env=QT_X11_NO_MITSHM=1 \
    --env=XAUTHORITY=/tmp/.docker.xauth \
    --env=ROS_LOCALHOST_ONLY=1 \
    --net=host \
    "$@"
