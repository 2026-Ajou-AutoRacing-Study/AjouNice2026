#!/usr/bin/env bash
# Record only: never launches or stops the driving stack.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
exec /usr/bin/python3 "$ROOT/scripts/rosbag/record/record_development_bag.py" "$@"
