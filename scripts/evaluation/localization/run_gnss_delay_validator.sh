#!/usr/bin/env bash
# The real validator owns its stack, recording and stop/P safety handling.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$ROOT/scripts/workspace/source_aw149.sh"
exec /usr/bin/python3 "$ROOT/scripts/evaluation/localization/run_gnss_delay_validator.py" "$@"
