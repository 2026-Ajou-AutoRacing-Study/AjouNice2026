#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
CONDA_ENV="${CONDA_ENV:-fastbev_rtx50}"
OUTPUT="$ROOT/artifacts/detection3d/fastbev/pretrained"
NUIM="$OUTPUT/cascade_mask_rcnn_r18_fpn_coco-mstrain_3x_20e_nuim_bbox_mAP_0.5110_segm_mAP_0.4070.pth"
NUS="$OUTPUT/fastbev_m0_r18_s256x704_v200x200x4_c192_d2_f4_epoch20_nuscenes.pth"
NUIM_SHA="fe9bedb99c28a4a02ac8215f0c9e5a31777eb17891e8fea7cf3becea825d39be"
NUS_SHA="db2326190537a777fdcf6c4f76950510b1219fc0ecc4beb1c9375fc99b377c65"

cd "$ROOT"
source "$HOME/miniconda3/etc/profile.d/conda.sh"
set +u
conda activate "$CONDA_ENV"
set -u

python -c 'import gdown' 2>/dev/null || python -m pip install gdown
mkdir -p "$OUTPUT"

if [[ ! -f "$NUIM" ]]; then
  gdown 1cXAX56lnCHwEUC0cTjP-gRK7zNrNz7Bc -O "$NUIM"
fi
if [[ ! -f "$NUS" ]]; then
  gdown 1_L3y6LMV9BAFJw0XaZRTNo-kDlhgTS17 -O "$NUS"
fi

printf '%s  %s\n%s  %s\n' \
  "$NUIM_SHA" "$NUIM" \
  "$NUS_SHA" "$NUS" \
  | sha256sum --check --strict

printf 'official initialization weights ready: %s\n' "$OUTPUT"
