#!/usr/bin/env bash
# Start the four-camera BEVFusion TCP server with the trusted local checkpoint.
set -euo pipefail

export PYTHONPATH="/opt/ajounice:/workspace/AjouNice2026:/opt/mmdetection3d:${PYTHONPATH:-}"
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

args=(
  python /opt/ajounice/app/inference_server.py
  --sensor-config "${SENSOR_CONFIG:-/config/Sensor.json}"
  --config "${MODEL_CONFIG:-/workspace/AjouNice2026/perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py}"
  --checkpoint "${CHECKPOINT:-/opt/checkpoints/bevfusion_base3.pth}"
  --bind "${BEVFUSION_BIND:-0.0.0.0}"
  --port "${BEVFUSION_PORT:-29500}"
  --intrinsic-source "${INTRINSIC_SOURCE:-horizontal_fov}"
  --camera-order camera_front camera_front_left camera_front_right camera_back
  --preprocess-workers "${PREPROCESS_WORKERS:-4}"
  --warmup-iterations "${WARMUP_ITERATIONS:-2}"
)

if [[ "${USE_AMP:-true}" == "true" ]]; then
  args+=(--amp)
fi

if [[ "${PROJECTION_ONLY:-false}" == "true" ]]; then
  args+=(--projection-only)
fi

if [[ "${TEMPORAL_FEATURE_CACHE:-false}" == "true" ]]; then
  args+=(--temporal-feature-cache)
  args+=(--temporal-maximum-age-sec "${TEMPORAL_MAXIMUM_AGE_SEC:-0.30}")
fi

exec "${args[@]}" "$@"
