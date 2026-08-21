#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 TAG [OUTPUT_DIR]" >&2
  exit 2
fi

tag="$1"
output_dir="${2:-$PWD/rosbags/raw_sensor_w_cam/perception_benchmark}"
mkdir -p "$output_dir"
output="$output_dir/$(date +%y%m%d_%H%M%S)_${tag}.bag"

echo "recording: $output"
echo "stop safely with Ctrl-C"
exec rosbag record --lz4 --buffsize=4096 -O "$output" \
  /clock \
  /Ego_topic \
  /Object_topic \
  /Competition_topic \
  /sensor/camera/front/compressed \
  /sensor/camera/front_left/compressed \
  /sensor/camera/front_right/compressed \
  /sensor/camera/back/compressed \
  /sensor/lidar1 \
  /sensor/gps \
  /sensor/imu \
  /CollisionData \
  /VehicleCollisionData \
  /IntscnTL_topic \
  /GetTrafficLightStatus \
  /ReplayInfo_topic \
  /lamps \
  /InsnControl \
  /InsnStatus \
  /SyncModeInfo \
  /sim/process/state/msg/MoraiSimProcStatus \
  /ctrl_cmd \
  /tf \
  /tf_static
