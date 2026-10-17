#!/usr/bin/env bash
# Manual CPU-only postprocessing after a zero-deficit 5K. Default is plan only.
# Never starts MORAI/ROS, installs dependencies, transfers, or deletes RAW PNG.
set -euo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ -e "$ROOT/checks/population_recovery_intent.json" || -L "$ROOT/checks/population_recovery_intent.json" ]]; then
  exec /usr/bin/python3 -B "$ROOT/checks/recovery_runtime.py" --workspace "$ROOT" finish "$@"
fi
if [[ -e "$ROOT/checks/route_compat_intent.json" || -L "$ROOT/checks/route_compat_intent.json" ]]; then
  exec /usr/bin/python3 -B "$ROOT/checks/live_route_compat.py" --workspace "$ROOT" finish "$@"
fi
exec /usr/bin/python3 -B "$ROOT/checks/finish_collection.py" --workspace "$ROOT" "$@"
