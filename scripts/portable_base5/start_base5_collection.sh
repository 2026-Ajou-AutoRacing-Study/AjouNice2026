#!/usr/bin/env bash
# Foreground terminal 3. Never runs on deployment; requires native host proof
# and the operator's --run --confirmed-environment sunny13 flags.
set -euo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec /usr/bin/python3 -B "$ROOT/checks/live_v2.py" --workspace "$ROOT" capture "$@"
