#!/usr/bin/env bash
# Start the Zivid One+ (SDK 2.11.1) container with USB + GPU access.
# Your home folder is shared with the container at the same path,
# so anything saved under your home folder in Zivid Studio is saved on the laptop.
# Default save folder: ~/zivid_oneplus_data
#
# Usage: ./run.sh                   -> interactive shell
#        ./run.sh ZividListCameras  -> run a single command
#        ./run.sh ZividStudio       -> GUI
set -e
mkdir -p "$HOME/zivid_oneplus_data"
xhost +local:docker >/dev/null 2>&1 || true
TTY=-i; [ -t 0 ] && TTY=-it
docker run --rm $TTY \
    --runtime=nvidia --gpus all \
    -v /dev:/dev \
    --device-cgroup-rule='c 189:* rmw' \
    --device-cgroup-rule='c 166:* rmw' \
    -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
    --user "$(id -u):$(id -g)" --group-add dialout \
    -e HOME="$HOME" -v "$HOME:$HOME" \
    -v "$HOME/zivid_oneplus_data:/data" \
    -w "$HOME/zivid_oneplus_data" \
    --name zivid-oneplus \
    zivid-oneplus:2.11.1 "$@"
