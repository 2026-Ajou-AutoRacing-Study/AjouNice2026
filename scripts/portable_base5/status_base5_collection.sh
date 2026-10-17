#!/usr/bin/env bash
# Read-only optional fourth terminal; does not start or stop the collector.
set -euo pipefail
umask 077
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec /usr/bin/python3 -B "$ROOT/checks/status.py" --workspace "$ROOT" "$@"
