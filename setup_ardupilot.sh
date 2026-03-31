#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

if git config -f .gitmodules --get-regexp '^submodule\.ardupilot\.path$' >/dev/null 2>&1; then
    echo "Initializing ArduPilot submodule..."
    git submodule update --init --recursive ardupilot
else
    if [ -d "ardupilot/.git" ] || [ -f "ardupilot/.git" ]; then
        echo "ArduPilot already exists at $ROOT_DIR/ardupilot"
        exit 0
    fi

    echo "Cloning ArduPilot repository..."
    git clone --recursive https://github.com/ArduPilot/ardupilot.git ardupilot
fi

echo "ArduPilot is ready at $ROOT_DIR/ardupilot"
