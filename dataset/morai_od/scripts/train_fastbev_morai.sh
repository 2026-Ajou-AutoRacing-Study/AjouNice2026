#!/usr/bin/env bash
set -euo pipefail

# Thin launcher: Fast-BEV's original tools/train.py remains the training engine.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONDA_ENV="${CONDA_ENV:-fastbev_rtx50}"
CONFIG="${FASTBEV_CONFIG:-$ROOT/dataset/morai_od/fastbev_plugin/configs/fastbev_m0_morai_4cam_s384x704.py}"
RUN_NAME="${1:-$(date +%y%m%d_%H%M%S)_fastbev_m0_morai_baseline}"
WORK_DIR="${FASTBEV_WORK_DIR:-$ROOT/artifacts/fastbev_runs/$RUN_NAME}"

cd "$ROOT"
source "$HOME/miniconda3/etc/profile.d/conda.sh"
set +u
conda activate "$CONDA_ENV"
set -u

export MMDET3D="$ROOT/third_party/fastbev"
export PYTHONPATH="$ROOT/third_party/fastbev:$ROOT/dataset/morai_od${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-fastbev}"
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

mkdir -p "$WORK_DIR"
printf 'config: %s\nwork_dir: %s\nselection_metric: center_mAP\n' \
  "$CONFIG" "$WORK_DIR"

python3 third_party/fastbev/tools/train.py \
  "$CONFIG" \
  --work-dir "$WORK_DIR" \
  --gpus 1 \
  --seed 0 \
  2>&1 | tee "$WORK_DIR/console.log"

printf 'training complete: %s\n' "$WORK_DIR"
find "$WORK_DIR" -maxdepth 1 -type f \
  -name 'best_center_mAP_epoch_*.pth' -print
