#!/usr/bin/env bash
# Start the four-camera BEVFusion TCP server with the trusted local checkpoint.
set -euo pipefail

export PYTHONPATH="/opt/ajounice:${PYTHONPATH:-}"
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

args=(
  python /opt/ajounice/app/inference_server.py
  --sensor-config "${SENSOR_CONFIG:-/config/Sensor_4.json}"
  --config "${MODEL_CONFIG:-/opt/mmdetection3d/projects/BEVFusion/configs/bevfusion_lidar-cam_voxel0075_second_secfpn_8xb4-cyclic-20e_nus-3d.py}"
  --checkpoint "${CHECKPOINT:-/opt/checkpoints/bevfusion_lidar_cam_nuscenes.pth}"
  --bind "${BEVFUSION_BIND:-0.0.0.0}"
  --port "${BEVFUSION_PORT:-29500}"
  --intrinsic-source "${INTRINSIC_SOURCE:-horizontal_fov}"
  --camera-order camera_front camera_front_left camera_front_right camera_back
)

if [[ "${PROJECTION_ONLY:-false}" == "true" ]]; then
  args+=(--projection-only)
fi

exec "${args[@]}" "$@"

