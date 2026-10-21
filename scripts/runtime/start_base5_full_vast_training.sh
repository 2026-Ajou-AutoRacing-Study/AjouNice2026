#!/usr/bin/env bash
# Full six-environment Base5 scope, never the completed 5K launcher.
# No arguments: read-only status. Actual training needs --run and a separately
# approved, data/code/epoch-bound start-authority file on the target.
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CONFIG="${BASE5_FULL_TRAINING_CONFIG:-$ROOT/artifacts/evaluation/base5_full30750_training_261021_v1/launcher_e12_ready.json}"

usage() {
  printf '%s\n' \
    'Usage: bash scripts/runtime/start_base5_full_vast_training.sh [--config PATH] [ACTION] [--resume]' \
    '  no action / --status: read-only remote status' \
    '  --check: online data/config/GPU/W&B/Drive preflight; no training' \
    '  --prepare: prepare a fresh or matching training contract; no training' \
    '  --smoke: CPU Concat dataset compatibility; no model execution' \
    '  --run: explicit training start, only after separate start approval' \
    '  --resume: use with an explicit action for the same frozen run'
}

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  usage
  exit 0
fi
if [[ "${1:-}" == --config ]]; then
  [[ $# -ge 2 ]] || { usage >&2; exit 2; }
  CONFIG="$2"
  shift 2
fi
if [[ "${1:-}" == --status ]]; then
  shift
  [[ $# == 0 ]] || { usage >&2; exit 2; }
fi
if [[ ! -f "$CONFIG" ]]; then
  printf 'Full-data launcher configuration is not ready: %s\n' "$CONFIG" >&2
  printf '%s\n' 'Do not substitute the old 5K launcher or an incomplete cohort index.' >&2
  exit 2
fi

exec python3 -B "$ROOT/scripts/runtime/full_data_oneclick.py" --config "$CONFIG" "$@"
