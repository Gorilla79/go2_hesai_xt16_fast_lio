#!/usr/bin/env bash
set -euo pipefail

WS="${HOME}/go2_ws"
PKG_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "${WS}/src"
rm -rf "${WS}/src/lidar_vision"
cp -a "${PKG_SRC}" "${WS}/src/lidar_vision"

cd "${WS}"
source /opt/ros/foxy/setup.bash

rm -rf build/lidar_vision install/lidar_vision
colcon build --packages-select lidar_vision --symlink-install

source install/setup.bash
ros2 pkg executables lidar_vision
