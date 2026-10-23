#!/usr/bin/env bash
# Read-only optional fourth terminal; does not start or stop the collector.
set -euo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ -e "$ROOT/checks/population_recovery_intent.json" || -L "$ROOT/checks/population_recovery_intent.json" ]]; then
  exec /usr/bin/python3 -B "$ROOT/checks/recovery_runtime.py" --workspace "$ROOT" status "$@"
fi
if [[ -e "$ROOT/checks/route_compat_intent.json" || -L "$ROOT/checks/route_compat_intent.json" ]]; then
  exec /usr/bin/python3 -B "$ROOT/checks/live_route_compat.py" --workspace "$ROOT" status "$@"
fi
exec /usr/bin/python3 -B "$ROOT/checks/status.py" --workspace "$ROOT" "$@"
