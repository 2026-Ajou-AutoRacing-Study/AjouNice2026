#!/usr/bin/env bash
# Full six-environment Base5 scope, never the completed 5K launcher.
# No arguments: read-only status. Actual training needs --run and a separately
# approved, data/code/epoch-bound start-authority file on the target.
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CONFIG="${BASE5_FULL_TRAINING_CONFIG:-$ROOT/artifacts/evaluation/base5_speed_repair_261021_v1/continuation_v2/launcher_speed_ready.json}"

usage() {
  printf '%s\n' \
    'Usage: bash scripts/runtime/start_base5_full_vast_training.sh [--config PATH] [ACTION] [--resume]' \
    '  no action / --status: read-only remote status' \
    '  --check: online data/config/GPU/W&B/Drive preflight; no training' \
    '  --prepare: prepare a fresh or matching training contract; no training' \
    '  --smoke: legacy full-data configuration CPU compatibility check only' \
    '  --run: explicit training start, only after separate start approval' \
    '  --resume: use with an explicit action for a checkpoint in the selected run' \
    '  performance transition: --run; later child checkpoint: --run --resume' \
    '  --config explicitly selects a known schema; no old-run fallback'
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

SCHEMA="$(python3 -B -c '
import json
from pathlib import Path
import sys
try:
    value = json.loads(Path(sys.argv[1]).read_text())
except (OSError, ValueError, UnicodeError) as error:
    print("Cannot read launcher schema: " + type(error).__name__, file=sys.stderr)
    raise SystemExit(2)
if not isinstance(value, dict) or not isinstance(value.get("schema"), str):
    print("Launcher configuration needs an explicit string schema.", file=sys.stderr)
    raise SystemExit(2)
print(value["schema"])
' "$CONFIG")" || exit 2

case "$SCHEMA" in
  base5_performance_launcher_v1)
    ENTRY="$ROOT/scripts/runtime/base5_performance_vast_training.py"
    ;;
  base5_full_data_launcher_v1)
    ENTRY="$ROOT/scripts/runtime/full_data_oneclick.py"
    ;;
  *)
    printf 'Unsupported full-data launcher schema: %s\n' "$SCHEMA" >&2
    printf '%s\n' 'Choose an explicit qualified configuration; no fallback is performed.' >&2
    exit 2
    ;;
esac

exec python3 -B "$ENTRY" --config "$CONFIG" "$@"
