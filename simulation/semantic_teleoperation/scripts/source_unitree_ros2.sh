#!/usr/bin/env bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unitree_root="$repo_root/external/unitree_ros2"
network_interface="${1:-lo}"

set +u
source /opt/ros/humble/setup.bash
source "$unitree_root/cyclonedds_ws/install/setup.bash"
if [[ -f "$unitree_root/example/install/setup.bash" ]]; then
  source "$unitree_root/example/install/setup.bash"
fi
source "$repo_root/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ "$network_interface" != "default" ]]; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$network_interface\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
else
  unset CYCLONEDDS_URI
fi

echo "Unitree ROS2 environment sourced with interface: $network_interface"
