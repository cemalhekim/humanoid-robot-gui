#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unitree_root="$repo_root/external/unitree_ros2"

if [[ ! -d "$unitree_root" ]]; then
  "$repo_root/scripts/fetch_unitree_ros2.sh"
fi

set +u
source /opt/ros/humble/setup.bash
set -u

colcon --log-base "$unitree_root/cyclonedds_ws/log" build \
  --base-paths "$unitree_root/cyclonedds_ws/src/unitree" \
  --build-base "$unitree_root/cyclonedds_ws/build" \
  --install-base "$unitree_root/cyclonedds_ws/install"

set +u
source "$unitree_root/cyclonedds_ws/install/setup.bash"
set -u

colcon --log-base "$unitree_root/example/log" build \
  --base-paths "$unitree_root/example/src" \
  --build-base "$unitree_root/example/build" \
  --install-base "$unitree_root/example/install"
