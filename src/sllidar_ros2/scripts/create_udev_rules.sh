#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing udev rules for SL lidars (front/rear)"
echo "Symlinks:"
echo "  /dev/ttyUSB_LIDAR1_FRONT"
echo "  /dev/ttyUSB_LIDAR2_REAR"
echo ""
echo "Copying ${SCRIPT_DIR}/rplidar.rules -> /etc/udev/rules.d/rplidar.rules"
sudo cp "${SCRIPT_DIR}/rplidar.rules" /etc/udev/rules.d/rplidar.rules

echo ""
echo "Reloading udev rules"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Done. Replug lidars if symlinks do not appear immediately."
