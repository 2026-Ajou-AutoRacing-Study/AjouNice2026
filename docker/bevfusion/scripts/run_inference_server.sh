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

if [[ "${BASE5_RUNTIME:-false}" == "true" ]]; then
  args+=(--base5-runtime --point-intensity-source "${POINT_INTENSITY_SOURCE:-ros255}")
fi
if [[ "${PARALLEL_CAMERA:-false}" == "true" ]]; then
  args+=(--parallel-camera)
fi

if [[ -n "${BEVFUSION_CAMERA_TENSORRT_POLICY:-}" ]]; then
  args+=(--camera-tensorrt-policy "$BEVFUSION_CAMERA_TENSORRT_POLICY")
fi
if [[ -n "${BEVFUSION_CAMERA_TENSORRT_ENGINE:-}" ]]; then
  args+=(--camera-tensorrt-engine "$BEVFUSION_CAMERA_TENSORRT_ENGINE")
fi
if [[ -n "${BEVFUSION_CAMERA_TENSORRT_METADATA:-}" ]]; then
  args+=(--camera-tensorrt-metadata "$BEVFUSION_CAMERA_TENSORRT_METADATA")
fi
case "${BEVFUSION_HEAD_ATTENTION_SDPA:-false}" in
  true) args+=(--head-attention-sdpa) ;;
  false) ;;
  *) echo '[ERROR] BEVFUSION_HEAD_ATTENTION_SDPA must be true or false.' >&2; exit 2 ;;
esac
case "${BEVFUSION_HEAD_CACHE_BEV_POS:-false}" in
  true) args+=(--head-cache-bev-pos) ;;
  false) ;;
  *) echo '[ERROR] BEVFUSION_HEAD_CACHE_BEV_POS must be true or false.' >&2; exit 2 ;;
esac
case "${POOL_FUSED_LAYOUT:-false}" in
  true) args+=(--pool-fused-layout) ;;
  false) ;;
  *) echo '[ERROR] POOL_FUSED_LAYOUT must be true or false.' >&2; exit 2 ;;
esac
if [[ -n "${CPU_THREADS:-}" ]]; then args+=(--cpu-threads "$CPU_THREADS"); fi
if [[ -n "${OPENCV_THREADS:-}" ]]; then args+=(--opencv-threads "$OPENCV_THREADS"); fi

if [[ "${PROJECTION_ONLY:-false}" == "true" ]]; then
  args+=(--projection-only)
fi

if [[ "${TEMPORAL_FEATURE_CACHE:-false}" == "true" ]]; then
  args+=(--temporal-feature-cache)
  args+=(--temporal-maximum-age-sec "${TEMPORAL_MAXIMUM_AGE_SEC:-0.30}")
fi

exec "${args[@]}" "$@"
