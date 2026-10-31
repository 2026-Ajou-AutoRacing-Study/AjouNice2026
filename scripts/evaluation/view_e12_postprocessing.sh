#!/usr/bin/env bash
# Saved E12 markers and original sensors only; never starts the live stack.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
source scripts/workspace/source_aw149.sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec /usr/bin/python3 scripts/evaluation/view_e12_postprocessing.py "$@"
