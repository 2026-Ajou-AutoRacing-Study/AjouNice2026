#!/usr/bin/env bash
set -euo pipefail

# Thin launcher: Fast-BEV's original tools/train.py remains the training engine.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
CONDA_ENV="${CONDA_ENV:-fastbev_rtx50}"
CONFIG="${FASTBEV_CONFIG:-$ROOT/perception/detection3d/fastbev/configs/fastbev_m0_morai_4cam_s384x704_i3_nuscenes_bev.py}"
RUN_NAME="${1:-$(date +%y%m%d_%H%M%S)_fastbev_m0_morai_i3}"
WORK_DIR="${FASTBEV_WORK_DIR:-$ROOT/artifacts/detection3d/fastbev/runs/$RUN_NAME}"
SEED="${FASTBEV_SEED:-0}"
if [[ $# -gt 0 ]]; then
  shift
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

# W&B credentials are managed by `wandb login`; never put an API key here.
# Reusing RUN_NAME intentionally resumes the same W&B run when local training
# is resumed.  Override any value from the shell when comparing experiments.
export WANDB_ENTITY="${WANDB_ENTITY:-AjouNice2026}"
export WANDB_PROJECT="${WANDB_PROJECT:-morai-fastbev}"
export WANDB_NAME="${WANDB_NAME:-$RUN_NAME}"
export WANDB_RUN_ID="${WANDB_RUN_ID:-$RUN_NAME}"
export WANDB_RESUME="${WANDB_RESUME:-allow}"
export WANDB_RUN_GROUP="${WANDB_RUN_GROUP:-sunny13-1500-i3-default}"
export WANDB_JOB_TYPE="${WANDB_JOB_TYPE:-train}"
export WANDB_TAGS="${WANDB_TAGS:-fastbev,morai,4cam,single-frame,s384x704,fp16,i3,nuscenes-pretrained}"
export WANDB_DIR="${WANDB_DIR:-$WORK_DIR}"
export WANDB_CACHE_DIR="${WANDB_CACHE_DIR:-$WORK_DIR/.wandb_cache}"

mkdir -p "$WORK_DIR" "$WANDB_CACHE_DIR"
printf 'config: %s\nwork_dir: %s\nseed: %s\nselection_metric: config evaluation.save_best\n' \
  "$CONFIG" "$WORK_DIR" "$SEED"

TRAIN_ARGS=(
  "$CONFIG"
  --work-dir "$WORK_DIR"
  --gpus 1
  --seed "$SEED"
  "$@"
)

if [[ "${FASTBEV_DISABLE_WANDB:-0}" == "1" ]]; then
  TRAIN_ARGS+=(--wo-wandb)
  printf 'wandb: disabled by FASTBEV_DISABLE_WANDB=1\n'
else
  python -c 'import wandb; print("wandb version:", wandb.__version__)'
  printf 'wandb: entity=%s project=%s run=%s group=%s mode=%s\n' \
    "$WANDB_ENTITY" "$WANDB_PROJECT" "$WANDB_NAME" \
    "$WANDB_RUN_GROUP" "${WANDB_MODE:-online}"
fi

python3 third_party/fastbev/tools/train.py \
  "${TRAIN_ARGS[@]}" \
  2>&1 | tee "$WORK_DIR/console.log"

printf 'training complete: %s\n' "$WORK_DIR"
find "$WORK_DIR" -maxdepth 1 -type f \
  \( -name 'best_center_mAP_epoch_*.pth' \
     -o -name 'best_merged_center_mAP_epoch_*.pth' \) -print
