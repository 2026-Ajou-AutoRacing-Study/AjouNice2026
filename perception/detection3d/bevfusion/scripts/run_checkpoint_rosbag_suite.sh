#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 CHECKPOINT OUTPUT_DIR [PLAYBACK_RATE_OVERRIDE]" >&2
  exit 2
fi

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
checkpoint="$(realpath "$1")"
output_dir="$(realpath -m "$2")"
playback_rate_override="${3:-}"
compose_file="$repo_root/docker/bevfusion/compose.yaml"
image="${BEVFUSION_IMAGE:-ajounice/bevfusion:team-cu128-v1}"

test -s "$checkpoint" || {
  echo "[ERROR] checkpoint is missing or empty: $checkpoint" >&2
  exit 1
}

export AJOUNICE_ROOT="$repo_root"
export BEVFUSION_CHECKPOINT="$checkpoint"
export BEVFUSION_IMAGE="$image"

cleanup() {
  docker compose -f "$compose_file" stop inference >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "[server] checkpoint=$checkpoint"
docker compose -f "$compose_file" up -d --force-recreate inference

ready=false
for _ in $(seq 1 90); do
  if PYTHONPATH="$repo_root/docker/bevfusion" \
      python3 "$repo_root/docker/bevfusion/scripts/healthcheck.py" \
      >/dev/null 2>&1; then
    ready=true
    break
  fi
  sleep 2
done
if [[ "$ready" != true ]]; then
  echo "[ERROR] BEVFusion inference server did not become ready" >&2
  docker compose -f "$compose_file" logs --tail=120 inference >&2 || true
  exit 1
fi

args=("$output_dir")
if [[ -n "$playback_rate_override" ]]; then
  args+=("$playback_rate_override")
fi
bash "$repo_root/perception/detection3d/bevfusion/scripts/run_base3_rosbag_suite.sh" \
  "${args[@]}"
