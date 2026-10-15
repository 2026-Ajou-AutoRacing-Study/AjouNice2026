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
case "$PUBLISH_FRAME_TIMING" in
  true|false) ;;
  *)
    echo "[ERROR] BEVFUSION_PUBLISH_FRAME_TIMING must be true or false." >&2
    exit 2
    ;;
esac
COMPOSE_FILE="$ROOT/docker/bevfusion/compose.yaml"
MODEL="${BEVFUSION_PRODUCTION_MODEL:-e24}"
if [[ "${1:-}" == "--model" ]]; then
  MODEL="${2:?Use --model e24 or --model e25}"
  shift 2
fi
if [[ "${1:-}" == "--publish-frame-timing" ]]; then
  PUBLISH_FRAME_TIMING=true
  shift
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
DETECTION_TOPIC="/perception/object_recognition/detection/objects"

export AJOUNICE_ROOT="$ROOT"
export BEVFUSION_CHECKPOINT="$CHECKPOINT"
export BEVFUSION_IMAGE="$IMAGE"
export BEVFUSION_SENSOR_CONFIG="$SENSOR_CONFIG"
export BEVFUSION_MODEL_CONFIG="/workspace/AjouNice2026/${profile[2]}"
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

if [[ -n "$(docker compose -f "$COMPOSE_FILE" ps --status running -q inference)" ]]; then
  echo "[ERROR] An inference service is already running. Stop its owning session before switching models." >&2
  exit 1
fi

echo "[BEVFusion] Starting Docker inference server"
echo "[BEVFusion] Production $MODEL; hash $EXPECTED_SHA256; sensor $SENSOR_CONFIG"
docker compose -f "$COMPOSE_FILE" up -d inference
server_owned=true

echo "[BEVFusion] Waiting for model readiness"
ready=false
for _ in $(seq 1 60); do
  if PYTHONPATH="$ROOT/docker/bevfusion" \
      python3 "$ROOT/docker/bevfusion/scripts/healthcheck.py" --expected-checkpoint "$EXPECTED_SHA256" \
      --expected-base5 "${profile[5]}" \
      --expected-config "$(sha256sum "$ROOT/${profile[2]}" | awk '{print $1}')" \
      --expected-sensor "$(sha256sum "$SENSOR_CONFIG" | awk '{print $1}')" \
      "${health_policy_args[@]}" >/dev/null 2>&1; then
    ready=true
    break
  fi
  sleep 2
done

if [[ "$ready" != "true" ]]; then
  echo "[ERROR] BEVFusion inference server did not become ready." >&2
  docker compose -f "$COMPOSE_FILE" logs --tail=100 inference >&2 || true
  exit 1
fi

source "$ROOT/scripts/workspace/source_aw149.sh"

echo "[BEVFusion] Starting ROS bridge on $DETECTION_TOPIC"
echo "[BEVFusion] Optional frame timing: $PUBLISH_FRAME_TIMING"
echo "[BEVFusion] Ctrl-C stops both the ROS bridge and Docker inference server"
if [[ -n "$SENSOR_CONFIG" ]]; then
  echo "[BEVFusion] Using sensor config: $SENSOR_CONFIG"
  roslaunch bevfusion_ros "${profile[4]}" sensor_config:="$SENSOR_CONFIG" \
    publish_frame_timing:="$PUBLISH_FRAME_TIMING"
else
  roslaunch bevfusion_ros "${profile[4]}" publish_frame_timing:="$PUBLISH_FRAME_TIMING"
fi
