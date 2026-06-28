#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$repo_root/base_ws/devel/setup.bash"

roslaunch bringup sensing.launch "$@"
