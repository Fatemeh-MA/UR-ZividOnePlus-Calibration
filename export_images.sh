#!/usr/bin/env bash
# Save both Docker images into one file, to move them to another computer without rebuilding.
# The file contains the Zivid SDK: keep it private (do not upload it to a public place).
#   ./export_images.sh [output.tar.gz]      (default: ./images/zivid-oneplus-images.tar.gz)
set -e
cd "$(dirname "$0")"
OUT="${1:-images/zivid-oneplus-images.tar.gz}"
mkdir -p "$(dirname "$OUT")"
docker save zivid-oneplus:2.11.1 zivid-oneplus-calib:2.11.1 | gzip -1 > "$OUT"
ls -lh "$OUT"
echo "On the other computer:  gunzip -c $(basename "$OUT") | docker load"
