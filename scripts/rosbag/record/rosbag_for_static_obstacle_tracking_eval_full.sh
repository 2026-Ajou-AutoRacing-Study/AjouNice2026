#!/usr/bin/env bash
set -euo pipefail

# Full-sensor counterpart of rosbag_for_static_obstacle_tracking_eval.sh.
# It preserves the same structured perception/planning/control topics and adds
# the four BEVFusion cameras, lidar1, GPS and IMU for detector replay.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export STATIC_OBSTACLE_RECORD_RAW_SENSORS=true

exec "$SCRIPT_DIR/rosbag_for_static_obstacle_tracking_eval.sh" \
  "${1:-bevfusion_static_obstacle_full}"
