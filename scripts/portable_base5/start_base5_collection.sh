#!/usr/bin/env bash
# Foreground terminal 3. Never runs on deployment; requires native host proof
# and the operator's --run --confirmed-environment sunny13 flags.
set -euo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ -e "$ROOT/checks/population_recovery_intent.json" || -L "$ROOT/checks/population_recovery_intent.json" ]]; then
  exec /usr/bin/python3 -B "$ROOT/checks/recovery_runtime.py" --workspace "$ROOT" capture "$@"
fi
if [[ -e "$ROOT/checks/route_compat_intent.json" || -L "$ROOT/checks/route_compat_intent.json" ]]; then
  exec /usr/bin/python3 -B "$ROOT/checks/live_route_compat.py" --workspace "$ROOT" capture "$@"
fi
exec /usr/bin/python3 -B "$ROOT/checks/live_v2.py" --workspace "$ROOT" capture "$@"
