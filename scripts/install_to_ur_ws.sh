#!/usr/bin/env bash
set -e
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$HOME/ur_ws/src"
rm -rf "$HOME/ur_ws/src/pbvs_camera"
cp -r "$PKG_DIR" "$HOME/ur_ws/src/pbvs_camera"
cd "$HOME/ur_ws"
colcon build --packages-select pbvs_camera --symlink-install
printf '\nDone. Now run:\n  source /opt/ros/jazzy/setup.bash\n  source ~/ur_ws/install/setup.bash\n'
