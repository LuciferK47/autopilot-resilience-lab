#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

if [ -d "ardupilot/.git" ] || [ -f "ardupilot/.git" ]; then
    echo "ArduPilot already exists at $ROOT_DIR/ardupilot"
    exit 0
fi

if [ -d "ardupilot" ] && [ -z "$(ls -A ardupilot)" ]; then
    rmdir ardupilot
fi

echo "Cloning ArduPilot repository..."
git clone --recursive https://github.com/ArduPilot/ardupilot.git ardupilot

echo "ArduPilot is ready at $ROOT_DIR/ardupilot"
