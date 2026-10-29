#!/usr/bin/env bash
# Source runtime/message definitions only; never start/stop nodes or send control.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECK_ARGS=("$@")
# Catkin setup scripts interpret caller arguments; do not pass our CLI to them.
set --
source "$ROOT/scripts/workspace/source_aw149.sh"
exec /usr/bin/python3 "$ROOT/scripts/runtime/check_sensor_inputs.py" "${CHECK_ARGS[@]}"
