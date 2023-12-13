#!/bin/bash

xhost +
docker run \
    -it \
    --runtime=nvidia \
    --rm \
    --network host \
    --volume=/tmp/.X11-unix/:/tmp/.X11-unix \
    --env="DISPLAY=$DISPLAY" \
    --volume /tmp/argus_socket:/tmp/argus_socket \
    --volume /etc/enctune.conf:/etc/enctune.conf \
    --volume /etc/nv_tegra_release:/etc/nv_tegra_release \
    --volume /tmp/nv_jetson_model:/tmp/nv_jetson_model \
    --volume $PWD/data:/data \
    --device /dev/snd \
    --device /dev/bus/usb \
    --privileged \
    "kinect:latest"

