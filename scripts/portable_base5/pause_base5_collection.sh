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
exec /usr/bin/python3 -B "$ROOT/checks/control.py" --workspace "$ROOT" "$ACTION" "$@"
