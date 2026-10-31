#!/usr/bin/env bash
# Independent fallback: standard ROS CLI only, no custom Python checker/overlay.
set -eo pipefail
item="${1:-help}"
if [[ "$item" == help || "$item" == --help ]]; then
  echo "usage: sensor_check_raw {network|vehicle|front|left|right|rear|lidar|gnss|imu}"
  exit 0
fi
if [[ "$item" == network ]]; then
  exec ip -br -4 addr
fi
case "$item" in
  vehicle) topic=/Competition_topic ;;
  front) topic=/sensor/camera/front/compressed ;;
  left) topic=/sensor/camera/front_left/compressed ;;
  right) topic=/sensor/camera/front_right/compressed ;;
  rear) topic=/sensor/camera/back/compressed ;;
  lidar) topic=/sensor/lidar1 ;;
  gnss) topic=/sensor/gps ;;
  imu) topic=/sensor/imu ;;
  *) echo "Unknown input: $item; use --help" >&2; exit 2 ;;
esac
# Do not forward CLI arguments to ROS setup scripts.
set --
source /opt/ros/noetic/setup.bash
echo "$topic : 5-second raw ROS rate check (read-only)"
set +e
timeout 5s rostopic hz "$topic"
status=$?
set -e
if [[ "$status" == 124 ]]; then
  echo "Measurement ended; check average rate / no new messages above."
  exit 0
fi
exit "$status"
