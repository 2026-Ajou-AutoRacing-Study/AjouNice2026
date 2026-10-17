#!/usr/bin/env bash
# Manual CPU-only postprocessing after a zero-deficit 5K. Default is plan only.
# Never starts MORAI/ROS, installs dependencies, transfers, or deletes RAW PNG.
set -euo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec /usr/bin/python3 -B "$ROOT/checks/finish_collection.py" --workspace "$ROOT" "$@"
