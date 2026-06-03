#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$repo_root/external"

if [[ -d "$repo_root/external/unitree_ros2/.git" ]]; then
  git -C "$repo_root/external/unitree_ros2" pull --ff-only
else
  git clone https://github.com/unitreerobotics/unitree_ros2.git "$repo_root/external/unitree_ros2"
fi
