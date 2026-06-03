#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$repo_root/external"

clone_or_update() {
  local url="$1"
  local branch="$2"
  local target="$3"

  if [[ -d "$target/.git" ]]; then
    git -C "$target" fetch --depth 1 origin "$branch"
    git -C "$target" checkout "$branch"
    git -C "$target" pull --ff-only
  else
    git clone --depth 1 --branch "$branch" "$url" "$target"
  fi
}

clone_or_update \
  https://github.com/unitreerobotics/unitree_mujoco.git \
  main \
  "$repo_root/external/unitree_mujoco"

clone_or_update \
  https://github.com/unitreerobotics/unitree_sdk2_python.git \
  master \
  "$repo_root/external/unitree_sdk2_python"
