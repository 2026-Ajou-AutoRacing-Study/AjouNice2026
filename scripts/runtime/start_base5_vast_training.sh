#!/usr/bin/env bash
# Local entrypoint: no conda/ROS activation; defaults start the remote run.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec /usr/bin/python3 "$ROOT/scripts/runtime/base5_vast_oneclick.py" "$@"
