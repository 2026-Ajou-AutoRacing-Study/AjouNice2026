#!/usr/bin/env bash
# Foreground terminal 2. Requires the ROS master owned by this workspace.
set -euo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec /usr/bin/python3 -B "$ROOT/checks/live_v2.py" --workspace "$ROOT" bridge "$@"
