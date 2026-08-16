#!/bin/bash
CONFIGFILE=docker/config/tools.sh
source $CONFIGFILE

xhost +local:docker

docker run -it --rm \
-e QT_X11_NO_MITSHM=1 \
--network=host \
--ipc=host \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-e DISPLAY=$DISPLAY \
-v $WORKSPACE_HOST:$WORKSPACE_CONTAINER \
--name $CONTAINER_NAME \
$IMAGE_NAME:$IMAGE_TAG