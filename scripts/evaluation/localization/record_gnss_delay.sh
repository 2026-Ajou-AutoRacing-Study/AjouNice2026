#!/usr/bin/env bash
# Observe an already-running full stack; never starts/stops its vehicle nodes.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$ROOT/scripts/workspace/source_aw149.sh"
exec /usr/bin/python3 "$ROOT/scripts/evaluation/localization/record_gnss_delay.py" "$@"
