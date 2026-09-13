#!/usr/bin/env bash
set -euo pipefail

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
image="${BEVFUSION_IMAGE:-ajounice/bevfusion:dev-cu128}"
info_root="$repo_root/artifacts/datasets/morai_base3_bevfusion_v1"
work_dir="${BEVFUSION_WORK_DIR:-$repo_root/artifacts/detection3d/bevfusion/training/base3_b0_4cam_lidar_e12}"
run_name="${WANDB_RUN_NAME:-bevfusion-base3-b0-4cam-lidar-e12}"

for split in train val test; do
  test -s "$info_root/morai_base3_infos_${split}.pkl" || {
    echo "Missing full info for ${split}; run prepare_base3_full_infos.sh first." >&2
    exit 2
  }
done
test -s "$repo_root/artifacts/detection3d/bevfusion/pretrained/bevfusion_nuscenes_morai3_compatible.pth" || {
  echo "Missing MORAI-adapted pretrained checkpoint; run run_base3_smoke.sh first." >&2
  exit 2
}
mkdir -p "$work_dir"

docker_args=(
  run --rm --gpus all --shm-size=12g
  --name ajounice-bevfusion-base3-train
  -v "$repo_root:/workspace/AjouNice2026"
  -v /home/luv/morai_cap:/home/luv/morai_cap:ro
  -w /workspace/AjouNice2026/perception/detection3d/bevfusion
  -e PYTHONPATH=/workspace/AjouNice2026:/opt/mmdetection3d
  -e TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
  -e WANDB_MODE="${WANDB_MODE:-online}"
  -e WANDB_DISABLE_CODE=true
)
if [[ -f "$HOME/.netrc" ]]; then
  docker_args+=( -v "$HOME/.netrc:/root/.netrc:ro" )
fi

container_work_dir="/workspace/AjouNice2026/${work_dir#"$repo_root/"}"
docker "${docker_args[@]}" "$image" \
  python /opt/mmdetection3d/tools/train.py \
  /workspace/AjouNice2026/perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py \
  --work-dir "$container_work_dir" \
  --cfg-options \
    visualizer.vis_backends.1.init_kwargs.name="$run_name"
