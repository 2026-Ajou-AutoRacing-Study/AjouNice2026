#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
TRAIN="$ROOT/perception/detection3d/fastbev/scripts/train.sh"
CONFIG_DIR="$ROOT/perception/detection3d/fastbev/configs"

if [[ "${1:-}" != "--execute" ]]; then
  printf 'This launches three 12-epoch W&B training runs.\n'
  printf 'Run after approval with: %s --execute\n' "$0"
  exit 2
fi

export WANDB_RUN_GROUP="sunny13-1500-initialization-phase1"
export WANDB_JOB_TYPE="initialization-ablation"

run_one() {
  local run_name="$1"
  local config="$2"
  local tags="$3"
  FASTBEV_CONFIG="$config" \
  WANDB_NAME="$run_name" \
  WANDB_RUN_ID="$run_name" \
  WANDB_TAGS="fastbev,morai,4cam,single-frame,s384x704,fp16,opencv,flip50-50,$tags" \
    "$TRAIN" "$run_name"
}

run_one \
  "init_i1_nuimages_backbone_seed0_12e" \
  "$CONFIG_DIR/fastbev_m0_morai_4cam_s384x704_i1_nuimages_backbone.py" \
  "i1,nuimages,backbone"

run_one \
  "init_i2_nuscenes_image_seed0_12e" \
  "$CONFIG_DIR/fastbev_m0_morai_4cam_s384x704_i2_nuscenes_image.py" \
  "i2,nuscenes,backbone-fpn"

run_one \
  "init_i3_nuscenes_bev_seed0_12e" \
  "$CONFIG_DIR/fastbev_m0_morai_4cam_s384x704_i3_nuscenes_bev.py" \
  "i3,nuscenes,backbone-fpn-bev"

printf 'initialization phase 1 complete\n'
