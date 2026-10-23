#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -f "$ROOT/.env.bevfusion.local" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT/.env.bevfusion.local"
  set +a
fi
# Optional sidecar diagnostics do not alter the frozen inference contract.
# Validate before any Docker operation, including readiness/image checks.
PUBLISH_FRAME_TIMING="${BEVFUSION_PUBLISH_FRAME_TIMING-false}"
SAFE_MODE="${BEVFUSION_SAFE_MODE:-false}"
case "$PUBLISH_FRAME_TIMING" in
  true|false) ;;
  *)
    echo "[ERROR] BEVFUSION_PUBLISH_FRAME_TIMING must be true or false." >&2
    exit 2
    ;;
esac
COMPOSE_FILE="$ROOT/docker/bevfusion/compose.yaml"
MODEL="${BEVFUSION_PRODUCTION_MODEL:-e24}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --model) MODEL="${2:?Use --model e24 or --model e25}"; shift 2 ;;
    --publish-frame-timing) PUBLISH_FRAME_TIMING=true; shift ;;
    --safe) SAFE_MODE=true; shift ;;
    --*) echo "[ERROR] Unknown BEVFusion option: $1" >&2; exit 2 ;;
    *) break ;;
  esac
done
case "$SAFE_MODE" in
  true|false) ;;
  *) echo '[ERROR] BEVFUSION_SAFE_MODE must be true or false.' >&2; exit 2 ;;
esac
if [[ $# -gt 1 ]]; then echo '[ERROR] At most one sensor config path is accepted.' >&2; exit 2; fi
if [[ "$SAFE_MODE" == true ]]; then
  # Same selected checkpoint/config and original AMP/parallel path, not a
  # different detector, GT fallback, hidden threshold or sensor change.
  export BEVFUSION_IMAGE=ajounice/bevfusion:team-cu128-v1
  export BEVFUSION_CAMERA_TENSORRT_POLICY=off
  unset BEVFUSION_CAMERA_TENSORRT_ENGINE BEVFUSION_CAMERA_TENSORRT_METADATA
  export BEVFUSION_POOL_FUSED_LAYOUT=false
  export BEVFUSION_CPU_THREADS=4 BEVFUSION_OPENCV_THREADS=1
  export BEVFUSION_PREPROCESS_WORKERS=4 BEVFUSION_WARMUP_ITERATIONS=2
fi
resolved="$(python3 "$ROOT/scripts/runtime/bevfusion_production_profile.py" --model "$MODEL" --fields)"
mapfile -t profile <<<"$resolved"
SENSOR_CONFIG="${1:-${BEVFUSION_SENSOR_CONFIG:-$ROOT/${profile[7]}}}"
SENSOR_CONFIG="$(realpath "$SENSOR_CONFIG")"
if [[ -n "$SENSOR_CONFIG" && ! -f "$SENSOR_CONFIG" ]]; then
  echo "[ERROR] BEVFusion sensor config not found: $SENSOR_CONFIG" >&2
  exit 1
fi
CHECKPOINT="${profile[0]}"
IMAGE="${BEVFUSION_IMAGE:-ajounice/bevfusion:team-cu128-v1}"
EXPECTED_SHA256="${profile[1]}"
health_policy_args=()
if [[ "$MODEL" == e24 ]]; then
  health_policy_args=(--require-box-origin-tag --expected-depth-raster-policy nearest_positive)
fi
if [[ "${BEVFUSION_CAMERA_TENSORRT_POLICY:-}" == require ]]; then
  health_policy_args+=(--expected-camera-backend camera_tensorrt)
elif [[ "$SAFE_MODE" == true || ( -z "${BEVFUSION_CAMERA_TENSORRT_ENGINE:-}" && -z "${BEVFUSION_CAMERA_TENSORRT_METADATA:-}" ) ]]; then
  health_policy_args+=(--expected-camera-backend eager)
fi
if [[ "$SAFE_MODE" == true ]]; then
  health_policy_args+=(--expected-pool-fused-layout false)
fi
DETECTION_TOPIC="/perception/object_recognition/detection/objects"

export AJOUNICE_ROOT="$ROOT"
export BEVFUSION_CHECKPOINT="$CHECKPOINT"
export BEVFUSION_IMAGE="$IMAGE"
export BEVFUSION_SENSOR_CONFIG="$SENSOR_CONFIG"
export BEVFUSION_MODEL_CONFIG="/workspace/AjouNice2026/${profile[2]}"
# The bridge uses horizontal_fov. A stale host environment must not create a
# ready server that rejects every request from that bridge.
export INTRINSIC_SOURCE=horizontal_fov
export BEVFUSION_BASE5_RUNTIME="${profile[5]}"
export BEVFUSION_PARALLEL_CAMERA="${profile[6]}"
export BEVFUSION_POINT_INTENSITY_SOURCE=native
if [[ "$MODEL" == e24 ]]; then export BEVFUSION_POINT_INTENSITY_SOURCE=ros255; fi
# Bridge/server port and precision are one frozen production contract.
export BEVFUSION_PORT=29500
export BEVFUSION_USE_AMP=true
export BEVFUSION_TEMPORAL_FEATURE_CACHE=false
export PROJECTION_ONLY=false
server_owned=false

check_server_identity() {
  PYTHONPATH="$ROOT/docker/bevfusion" \
    python3 "$ROOT/docker/bevfusion/scripts/healthcheck.py" --expected-checkpoint "$EXPECTED_SHA256" \
    --expected-base5 "${profile[5]}" \
    --expected-config "$(sha256sum "$ROOT/${profile[2]}" | awk '{print $1}')" \
    --expected-sensor "$(sha256sum "$SENSOR_CONFIG" | awk '{print $1}')" \
    --expected-intrinsic-source horizontal_fov \
    --expected-camera-order camera_front camera_front_left camera_front_right camera_back \
    "${health_policy_args[@]}"
}

wait_for_selected_server() {
  for _ in $(seq 1 60); do
    if check_server_identity >/dev/null 2>&1; then ready=true; return 0; fi
    sleep 2
  done
  return 1
}

cleanup() {
  if [[ "$server_owned" == true ]]; then
    docker compose -f "$COMPOSE_FILE" stop inference >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if [[ ! -f "$CHECKPOINT" ]]; then
  echo "[ERROR] BEVFusion checkpoint not found: $CHECKPOINT" >&2
  exit 1
fi

actual_sha256="$(sha256sum "$CHECKPOINT" | awk '{print $1}')"
if [[ "$actual_sha256" != "$EXPECTED_SHA256" ]]; then
  echo "[ERROR] BEVFusion checkpoint SHA-256 mismatch" >&2
  echo "  expected: $EXPECTED_SHA256" >&2
  echo "  actual:   $actual_sha256" >&2
  echo "Check BEVFUSION_CHECKPOINT overrides; rollback is --model e25." >&2
  exit 1
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[ERROR] Docker image is missing: $IMAGE" >&2
  echo "  Build it from $ROOT before starting BEVFusion." >&2
  exit 1
fi

if topic_info="$(rostopic info "$DETECTION_TOPIC" 2>/dev/null)"; then
  existing_publishers="$(awk '
    /^Publishers:/ { in_publishers=1; next }
    /^Subscribers:/ { in_publishers=0 }
    in_publishers && /^ \* / { print }
  ' <<<"$topic_info")"
  if [[ -n "$existing_publishers" ]]; then
    echo "[ERROR] Another detector already publishes $DETECTION_TOPIC:" >&2
    echo "$existing_publishers" >&2
    echo "Stop PointPainting, FastBEV, playback, or another BEVFusion bridge first." >&2
    exit 1
  fi
fi

ready=false
if [[ -n "$(docker compose -f "$COMPOSE_FILE" ps --status running -q inference)" ]]; then
  # Docker may restart an already-warmed eager server when the PC boots. It
  # need not block a fresh bridge if every selected frozen identity matches.
  # Never take ownership, change its options, or kill someone else's server.
  if [[ -n "${BEVFUSION_CAMERA_TENSORRT_ENGINE:-}${BEVFUSION_CAMERA_TENSORRT_METADATA:-}" ]]; then
    echo '[ERROR] An inference service is already running; stop its owning session before changing acceleration options.' >&2
    exit 1
  fi
  echo '[BEVFusion] Waiting for the existing server to finish warmup and match the selected profile'
  if wait_for_selected_server; then
    echo '[BEVFusion] Reusing the already-ready matching eager server; this session will not stop it.'
  else
    echo '[ERROR] The existing inference service did not become ready with the selected profile.' >&2
    check_server_identity >&2 || true
    echo 'Stop its owning session before switching; no unrelated detector is killed automatically.' >&2
    exit 1
  fi
else
  echo "[BEVFusion] Starting Docker inference server"
  echo "[BEVFusion] Production $MODEL; hash $EXPECTED_SHA256; sensor $SENSOR_CONFIG"
  echo "[BEVFusion] Safe mode: $SAFE_MODE; camera TRT policy: ${BEVFUSION_CAMERA_TENSORRT_POLICY:-automatic off/prefer}"
  docker compose -f "$COMPOSE_FILE" up -d inference
  server_owned=true
  echo "[BEVFusion] Waiting for model readiness"
  wait_for_selected_server || true
fi

if [[ "$ready" != "true" ]]; then
  echo "[ERROR] BEVFusion inference server did not become ready." >&2
  docker compose -f "$COMPOSE_FILE" logs --tail=100 inference >&2 || true
  exit 1
fi

source "$ROOT/scripts/workspace/source_aw149.sh"

echo "[BEVFusion] Starting ROS bridge on $DETECTION_TOPIC"
echo "[BEVFusion] Optional frame timing: $PUBLISH_FRAME_TIMING"
if [[ "$server_owned" == true ]]; then
  echo "[BEVFusion] Ctrl-C stops both the ROS bridge and Docker inference server"
else
  echo "[BEVFusion] Ctrl-C stops this ROS bridge; the pre-existing matching server is not owned by this session"
fi
if [[ -n "$SENSOR_CONFIG" ]]; then
  echo "[BEVFusion] Using sensor config: $SENSOR_CONFIG"
  roslaunch bevfusion_ros "${profile[4]}" sensor_config:="$SENSOR_CONFIG" \
    publish_frame_timing:="$PUBLISH_FRAME_TIMING"
else
  roslaunch bevfusion_ros "${profile[4]}" publish_frame_timing:="$PUBLISH_FRAME_TIMING"
fi
