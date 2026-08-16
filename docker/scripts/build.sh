#!/bin/bash

CONFIGFILE=docker/config/tools.sh
source "$CONFIGFILE"

docker build \
    --network=host \
    -f "$DOCKERFILE" \
    -t "$IMAGE_NAME:$IMAGE_TAG" \
    --rm \
    .