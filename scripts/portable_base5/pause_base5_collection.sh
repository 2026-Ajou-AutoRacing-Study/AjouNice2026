#!/usr/bin/env bash
# Default is read-only. --run requests a safe bounded-chunk pause.
# --resume-paused --run only archives PAUSE after exit evidence checks;
# the operator must separately run start_base5_collection.sh again.
set -euo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ACTION=pause
if [[ "${1:-}" == --resume-paused ]]; then
  ACTION=resume-paused
  shift
fi
if [[ -e "$ROOT/checks/population_recovery_intent.json" || -L "$ROOT/checks/population_recovery_intent.json" ]]; then
  exec /usr/bin/python3 -B "$ROOT/checks/recovery_runtime.py" --workspace "$ROOT" "$ACTION" "$@"
fi
if [[ -e "$ROOT/checks/route_compat_intent.json" || -L "$ROOT/checks/route_compat_intent.json" ]]; then
  exec /usr/bin/python3 -B "$ROOT/checks/live_route_compat.py" --workspace "$ROOT" "$ACTION" "$@"
fi
exec /usr/bin/python3 -B "$ROOT/checks/control.py" --workspace "$ROOT" "$ACTION" "$@"
