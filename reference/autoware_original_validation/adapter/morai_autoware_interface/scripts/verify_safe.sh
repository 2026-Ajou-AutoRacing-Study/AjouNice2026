#!/usr/bin/env bash

set -eo pipefail

verification_workspace=/home/ajou/autoware-0.51

source /opt/ros/humble/setup.bash
source "${verification_workspace}/install/setup.bash"

verification_launch_pid=''

cleanup_verification() {
  if [[ -n "${verification_launch_pid}" ]] && kill -0 "${verification_launch_pid}" 2>/dev/null; then
    kill -TERM "${verification_launch_pid}" 2>/dev/null || true
    wait "${verification_launch_pid}" 2>/dev/null || true
  fi
}

trap cleanup_verification EXIT INT TERM

ros2 launch morai_autoware_interface safe_verification_adapter.launch.py &
verification_launch_pid=$!

sleep 2
ros2 run morai_autoware_interface morai_autoware_verifier
