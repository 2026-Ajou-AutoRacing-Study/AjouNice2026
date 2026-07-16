#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/noetic/setup.bash

mkdir -p "$REPO_ROOT/base_ws/src"
cd "$REPO_ROOT/base_ws"

catkin_make -DCMAKE_BUILD_TYPE=Release

echo "Workspace initialized."
