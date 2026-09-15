#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -f "$ROOT/.env.bevfusion.local" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT/.env.bevfusion.local"
  set +a
fi
COMPOSE_FILE="$ROOT/docker/bevfusion/compose.yaml"
CHECKPOINT="${BEVFUSION_CHECKPOINT:-${BEVFUSION_INFERENCE_CHECKPOINT:-$HOME/bevfusion_artifacts/checkpoints/base3_bf0_e10_inference.pth}}"
IMAGE="${BEVFUSION_IMAGE:-ajounice/bevfusion:team-cu128-v1}"
EXPECTED_SHA256="b2d3f65befaa16382cdd189f9c8ac1aff2d2176e6c48e19a17ab329947eff419"
DETECTION_TOPIC="/perception/object_recognition/detection/objects"

export AJOUNICE_ROOT="$ROOT"
export BEVFUSION_CHECKPOINT="$CHECKPOINT"
export BEVFUSION_IMAGE="$IMAGE"

cleanup() {
  docker compose -f "$COMPOSE_FILE" stop inference >/dev/null 2>&1 || true
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

echo "[BEVFusion] Starting Docker inference server"
docker compose -f "$COMPOSE_FILE" up -d inference

echo "[BEVFusion] Waiting for model readiness"
ready=false
for _ in $(seq 1 60); do
  if PYTHONPATH="$ROOT/docker/bevfusion" \
      python3 "$ROOT/docker/bevfusion/scripts/healthcheck.py" >/dev/null 2>&1; then
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
echo "[BEVFusion] Ctrl-C stops both the ROS bridge and Docker inference server"
roslaunch bevfusion_ros base3_online.launch
