#!/usr/bin/env bash
# One-time host setup for the Zivid One+ (run with: sudo ./host/setup_host.sh).
# Does NOT install any Zivid SDK on the host - the SDK lives only in the Docker image.
#   - USB permissions for the 3 USB devices inside the One+ (controller, projector, image sensor)
#   - adds the calling user to 'dialout' (the camera controller is also a serial device)
set -e
if [ "$(id -u)" -ne 0 ]; then echo "run with sudo"; exit 1; fi
HERE="$(cd "$(dirname "$0")" && pwd)"
install -m 644 "$HERE/99-zivid.rules" /etc/udev/rules.d/99-zivid.rules
udevadm control --reload-rules && udevadm trigger
TARGET_USER="${SUDO_USER:-$USER}"
usermod -aG dialout "$TARGET_USER"
echo "Done. Log out and back in (for the dialout group), then unplug/replug the camera."
echo "Check: docker, NVIDIA driver and nvidia-container-toolkit must be installed (see README)."
