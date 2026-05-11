#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/../../catkin_ws/devel/setup.bash"

roslaunch ajn_bringup full_stack.launch