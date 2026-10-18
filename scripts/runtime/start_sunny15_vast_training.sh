#!/usr/bin/env bash
# New Sunny15 only. No arguments after variant = read-only status, never train.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
variant="${1:-}"
case "$variant" in
  cosine|restarts) shift ;;
  *) echo "Usage: bash $0 cosine|restarts [--status|--prepare|--check|--run [--resume]]" >&2; exit 2 ;;
esac
config_root="${SUNNY15_LAUNCH_CONFIG_ROOT:-$ROOT/artifacts/evaluation/bevfusion/sunny15_training_readiness_261018_v5}"
exec /usr/bin/python3 -B "$ROOT/scripts/runtime/sunny15_vast_training.py" \
  --config "$config_root/$variant.json" "$@"
