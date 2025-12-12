#!/usr/bin/env bash
set -euo pipefail
sudo apt update
sudo apt install -y ros-humble-gazebo-*
sudo apt install -y ros-humble-cartographer ros-humble-cartographer-ros
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-vision-msgs ros-humble-cv-bridge
echo "Done. TB3 deps installed."
