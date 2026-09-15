#!/usr/bin/env bash
set -euo pipefail

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
capture_root="${BASE3_CAPTURE_ROOT:?Set BASE3_CAPTURE_ROOT to the extracted Base3 capture root}"
metadata_root="${BASE3_METADATA_ROOT:-$repo_root/artifacts/datasets/morai_base3_static_10k_pointpainting_input_v1}"
smoke_info="$repo_root/artifacts/datasets/morai_base3_bevfusion_2080ti_smoke"

python3 "$repo_root/perception/detection3d/bevfusion/tools/build_base3_bevfusion_infos.py" \
  --dataset-root "$metadata_root" \
  --output-dir "$smoke_info" \
  --capture-root "$capture_root" \
  --serialized-capture-root /datasets/base3 \
  --minimum-gt-points 5 \
  --box-tolerance-m 0.03 \
  --max-frames-per-split 8

BASE3_INFO_ROOT="$smoke_info" \
BEVFUSION_EPOCHS=1 \
BEVFUSION_WORK_DIR="$repo_root/artifacts/detection3d/bevfusion/training/2080ti_handoff_smoke" \
WANDB_MODE="${WANDB_MODE:-disabled}" \
WANDB_RUN_NAME="bevfusion-base3-2080ti-handoff-smoke" \
  "$repo_root/perception/detection3d/bevfusion/scripts/run_base3_2080ti_training.sh" fresh
