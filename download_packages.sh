#!/usr/bin/env bash
# Download the Zivid SDK 2.11.1 packages (Ubuntu 22.04) needed to build the Docker image.
# They are NOT stored in git (Zivid proprietary software, ~110 MB).
# Source: https://www.zivid.com/downloads/revisions/2.11.1  (last SDK that supports the Zivid One+)
set -e
cd "$(dirname "$0")"
BASE=https://downloads.zivid.com/sdk/releases/2.11.1+de9b5dae-1/u22
for f in \
    zivid-telicam-driver_3.0.1.1-3_amd64.deb \
    zivid_2.11.1+de9b5dae-1_amd64.deb \
    zivid-studio_2.11.1+de9b5dae-1_amd64.deb \
    zivid-tools_2.11.1+de9b5dae-1_amd64.deb; do
    if [ -f "$f" ]; then echo "already have $f"; continue; fi
    echo "downloading $f"
    curl -fL -o "$f" "$BASE/$f"
done
ls -lh ./*.deb
