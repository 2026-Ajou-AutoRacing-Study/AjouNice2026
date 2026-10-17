#!/usr/bin/env bash
# Foreground terminal 1. Add --run explicitly; default only prints the plan.
set -euo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec /usr/bin/python3 -B "$ROOT/checks/live_v2.py" --workspace "$ROOT" ros "$@"
