#!/usr/bin/env bash
set -euo pipefail

# Evaluate the validation-selected best checkpoint on the held-out test split.
if [[ $# -ne 1 ]]; then
  echo "usage: $0 <fastbev-work-dir>" >&2
  exit 2
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
WORK_DIR="$(realpath "$1")"
CONDA_ENV="${CONDA_ENV:-fastbev_rtx50}"
CONFIG="${FASTBEV_CONFIG:-$ROOT/perception/detection3d/fastbev/configs/fastbev_m0_morai_4cam_s384x704.py}"
CHECKPOINT="$(find "$WORK_DIR" -maxdepth 1 -type f \
  -name 'best_center_mAP_epoch_*.pth' -print | sort -V | tail -n 1)"

if [[ -z "$CHECKPOINT" ]]; then
  echo "best center_mAP checkpoint not found in: $WORK_DIR" >&2
  exit 1
fi

cd "$ROOT"
source "$HOME/miniconda3/etc/profile.d/conda.sh"
set +u
conda activate "$CONDA_ENV"
set -u

export MMDET3D="$ROOT/third_party/fastbev"
export PYTHONPATH="$ROOT:$ROOT/third_party/fastbev${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-fastbev}"
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

printf 'config: %s\ncheckpoint: %s\nsplit: test\n' "$CONFIG" "$CHECKPOINT"

EVAL_ARGS=()
if [[ "${FASTBEV_DISABLE_WANDB:-0}" != "1" ]]; then
  export WANDB_ENTITY="${WANDB_ENTITY:-AjouNice2026}"
  export WANDB_PROJECT="${WANDB_PROJECT:-morai-fastbev}"
  export WANDB_RUN_ID="${WANDB_RUN_ID:-$(basename "$WORK_DIR")}"
  EVAL_ARGS+=(--wandb --wandb-run-id "$WANDB_RUN_ID")
  printf 'wandb test logging: %s/%s run=%s\n' \
    "$WANDB_ENTITY" "$WANDB_PROJECT" "$WANDB_RUN_ID"
fi

python3 perception/detection3d/fastbev/tools/evaluate.py \
  --config "$CONFIG" \
  --checkpoint "$CHECKPOINT" \
  --fp16 \
  --cudnn-benchmark \
  --metrics center,tp,iou \
  --output-json "$WORK_DIR/test_metrics.json" \
  "${EVAL_ARGS[@]}" \
  2>&1 | tee "$WORK_DIR/test_evaluation.log"
