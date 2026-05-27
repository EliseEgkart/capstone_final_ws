#!/bin/bash

set -euo pipefail

echo "Removing /etc/udev/rules.d/rplidar.rules"
sudo rm -f /etc/udev/rules.d/rplidar.rules

echo ""
echo "Reloading udev rules"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Done."
