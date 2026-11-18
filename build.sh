#!/usr/bin/env bash
# One command to create both Docker images:
#   zivid-oneplus:2.11.1        Zivid SDK 2.11.1 + USB driver + Zivid Studio
#   zivid-oneplus-calib:2.11.1  + zivid-python, OpenCV, UR RTDE client (calibration scripts)
# Needs internet the first time (~10-15 min). Safe to run again: finished steps are cached.
set -e
cd "$(dirname "$0")"

command -v docker >/dev/null || { echo "ERROR: docker is not installed (see README, section 1)"; exit 1; }
docker info >/dev/null 2>&1 || { echo "ERROR: cannot talk to docker. Is it running? Are you in the 'docker' group (log out/in)?"; exit 1; }
docker info 2>/dev/null | grep -q -i 'runtimes.*nvidia' \
    || echo "WARNING: docker has no 'nvidia' runtime. Install the NVIDIA Container Toolkit (README, section 1), or the camera will not work."

echo "== 1/3  Zivid SDK packages =="
./download_packages.sh

echo "== 2/3  image zivid-oneplus:2.11.1 =="
docker build -t zivid-oneplus:2.11.1 .

echo "== 3/3  image zivid-oneplus-calib:2.11.1 =="
docker build -t zivid-oneplus-calib:2.11.1 calib/

echo
docker images | grep -E 'REPOSITORY|zivid-oneplus'
echo
echo "Done. Plug in the camera and run:  ./run.sh ZividListCameras"
