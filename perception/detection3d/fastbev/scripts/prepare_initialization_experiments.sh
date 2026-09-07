#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
CONDA_ENV="${CONDA_ENV:-fastbev_rtx50}"
RAW="$ROOT/artifacts/detection3d/fastbev/pretrained"
PREPARED="$RAW/prepared"
REPORTS="$RAW/reports"
TOOL="$ROOT/perception/detection3d/fastbev/tools/prepare_initialization_checkpoint.py"
CONFIG="$ROOT/perception/detection3d/fastbev/configs/fastbev_m0_morai_4cam_s384x704.py"
NUIM="$RAW/cascade_mask_rcnn_r18_fpn_coco-mstrain_3x_20e_nuim_bbox_mAP_0.5110_segm_mAP_0.4070.pth"
NUS="$RAW/fastbev_m0_r18_s256x704_v200x200x4_c192_d2_f4_epoch20_nuscenes.pth"

cd "$ROOT"
source "$HOME/miniconda3/etc/profile.d/conda.sh"
set +u
conda activate "$CONDA_ENV"
set -u

export MMDET3D="$ROOT/third_party/fastbev"
export PYTHONPATH="$ROOT:$ROOT/third_party/fastbev${PYTHONPATH:+:$PYTHONPATH}"
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-fastbev-init}"

mkdir -p "$PREPARED" "$REPORTS"

python3 "$TOOL" \
  --source "$NUIM" \
  --output "$PREPARED/i1_nuimages_backbone.pth" \
  --report "$REPORTS/i1_nuimages_backbone.json" \
  --config "$CONFIG" \
  --mode backbone \
  --label i1_nuimages_backbone

python3 "$TOOL" \
  --source "$NUS" \
  --output "$PREPARED/i2_nuscenes_image.pth" \
  --report "$REPORTS/i2_nuscenes_image.json" \
  --config "$CONFIG" \
  --mode image \
  --label i2_nuscenes_image

python3 "$TOOL" \
  --source "$NUS" \
  --output "$PREPARED/i3_nuscenes_image_bev.pth" \
  --report "$REPORTS/i3_nuscenes_image_bev.json" \
  --config "$CONFIG" \
  --mode image_bev \
  --label i3_nuscenes_image_bev

sha256sum \
  "$NUIM" \
  "$NUS" \
  "$PREPARED/i1_nuimages_backbone.pth" \
  "$PREPARED/i2_nuscenes_image.pth" \
  "$PREPARED/i3_nuscenes_image_bev.pth" \
  > "$REPORTS/sha256sums.txt"
printf 'initialization checkpoints prepared: %s\n' "$PREPARED"
