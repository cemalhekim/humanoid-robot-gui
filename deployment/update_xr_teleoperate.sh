#!/usr/bin/env bash
set -euo pipefail

XR_DIR="${XR_TELEOPERATE_DIR:-/home/unitree/xr_teleoperate}"
BRANCH="${XR_TELEOPERATE_BRANCH:-main}"

if [[ ! -d "$XR_DIR/.git" ]]; then
  echo "xr_teleoperate repo not found at $XR_DIR" >&2
  exit 0
fi

cd "$XR_DIR"

if ! git fetch origin "$BRANCH"; then
  echo "Could not fetch xr_teleoperate origin/$BRANCH" >&2
  exit 0
fi

old_head="$(git rev-parse HEAD)"
new_head="$(git rev-parse "origin/$BRANCH")"

if [[ "$old_head" == "$new_head" ]]; then
  exit 0
fi

echo "Updating xr_teleoperate from ${old_head:0:7} to ${new_head:0:7}"

if git submodule status --recursive >/dev/null 2>&1; then
  git submodule foreach --recursive 'git reset --hard >/dev/null 2>&1 || true'
fi

git reset --hard "origin/$BRANCH"
git submodule sync --recursive
git submodule update --init --recursive --depth 1

exit 10
