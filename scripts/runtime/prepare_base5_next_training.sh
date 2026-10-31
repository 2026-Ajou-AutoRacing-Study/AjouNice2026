#!/usr/bin/env bash
# Run INSIDE the existing BEVFusion Python environment. Never SSH or train.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "${BASE5_TRAINING_PYTHON:-python3}" \
  "$ROOT/perception/detection3d/bevfusion/tools/prepare_base5_next_training.py" "$@"
