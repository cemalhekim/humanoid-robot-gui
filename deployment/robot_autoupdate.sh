#!/usr/bin/env bash
set -euo pipefail

cd /home/unitree/robot_telemetry_web

if ! git fetch origin main; then
  exit 0
fi

old="$(git rev-parse HEAD)"
new="$(git rev-parse origin/main)"

if [ "$old" = "$new" ]; then
  exit 0
fi

git checkout -B main origin/main
git reset --hard origin/main
python3 run_servers.py --mode systemd
