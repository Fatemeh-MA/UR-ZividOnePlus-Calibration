#!/usr/bin/env bash
# Run a command in the Zivid One+ calibration container (SDK 2.11.1 + zivid-python 2.11.1).
# This folder is mounted as /work, so datasets/ and results are saved here on the host.
#
#   ./calib.sh python3 test_capture.py
#   ./calib.sh python3 universal_robots_perform_hand_eye_calibration.py --eth --ip 192.168.15.10 checkerboard
#   ./calib.sh bash
set -e
cd "$(dirname "$0")"
xhost +local:docker >/dev/null 2>&1 || true
TTY=-i; [ -t 0 ] && TTY=-it
docker run --rm $TTY \
    --runtime=nvidia --gpus all \
    --network host \
    -v /dev:/dev \
    --device-cgroup-rule='c 189:* rmw' \
    --device-cgroup-rule='c 166:* rmw' \
    -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD:/work" -w /work \
    -v "$HOME:$HOME" \
    --user "$(id -u):$(id -g)" -e HOME=/tmp \
    zivid-oneplus-calib:2.11.1 "${@:-bash}"
