#!/usr/bin/env bash
set -euo pipefail

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
image="${BEVFUSION_IMAGE:-ajounice/bevfusion:dev-cu128}"
source_root="${BASE3_SOURCE_ROOT:-$repo_root/artifacts/datasets/morai_base3_static_10k_pointpainting_input_v1}"
info_root="$repo_root/artifacts/datasets/morai_base3_bevfusion_smoke_v1"
work_dir="$repo_root/artifacts/detection3d/bevfusion/training/base3_b0_smoke"
checkpoint="${BEVFUSION_NUSCENES_CHECKPOINT:-/home/luv/bevfusion_artifacts/checkpoints/bevfusion_lidar_cam_nuscenes.pth}"

python3 "$repo_root/perception/detection3d/bevfusion/tools/build_base3_bevfusion_infos.py" \
  --dataset-root "$source_root" \
  --output-dir "$info_root" \
  --minimum-gt-points 5 \
  --box-tolerance-m 0.03 \
  --max-frames-per-split "${SMOKE_FRAMES_PER_SPLIT:-16}"

mkdir -p "$work_dir"
pretrained_dir="$repo_root/artifacts/detection3d/bevfusion/pretrained"
mkdir -p "$pretrained_dir"
adapter="$repo_root/perception/detection3d/bevfusion/tools/adapt_nuscenes_checkpoint.py"
adapted_checkpoint="$pretrained_dir/bevfusion_nuscenes_morai3_compatible.pth"
if [[ ! -s "$adapted_checkpoint" || "$adapter" -nt "$adapted_checkpoint" || "$checkpoint" -nt "$adapted_checkpoint" ]]; then
  docker run --rm \
    -v "$repo_root:/workspace/AjouNice2026" \
    -v "$checkpoint:/opt/checkpoints/bevfusion_lidar_cam_nuscenes.pth:ro" \
    "$image" \
    python /workspace/AjouNice2026/perception/detection3d/bevfusion/tools/adapt_nuscenes_checkpoint.py \
      --input /opt/checkpoints/bevfusion_lidar_cam_nuscenes.pth \
      --output /workspace/AjouNice2026/artifacts/detection3d/bevfusion/pretrained/bevfusion_nuscenes_morai3_compatible.pth
fi
docker_args=(
  run --rm --gpus all --shm-size=8g
  --name ajounice-bevfusion-base3-smoke
  -v "$repo_root:/workspace/AjouNice2026"
  -v /home/luv/morai_cap:/home/luv/morai_cap:ro
  -v "$checkpoint:/opt/checkpoints/bevfusion_lidar_cam_nuscenes.pth:ro"
  -w /workspace/AjouNice2026/perception/detection3d/bevfusion
  -e AJOUNICE_ROOT=/workspace/AjouNice2026
  -e BASE3_BEVFUSION_INFO_ROOT=/workspace/AjouNice2026/artifacts/datasets/morai_base3_bevfusion_smoke_v1
  -e BEVFUSION_NUSCENES_CHECKPOINT=/opt/checkpoints/bevfusion_lidar_cam_nuscenes.pth
  -e PYTHONPATH=/workspace/AjouNice2026:/opt/mmdetection3d
  -e TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
  -e WANDB_PROJECT="${WANDB_PROJECT:-morai-3d-object-detection}"
  -e WANDB_ENTITY="${WANDB_ENTITY:-AjouNice2026}"
  -e WANDB_RUN_NAME="${WANDB_RUN_NAME:-bevfusion-base3-b0-smoke}"
  -e WANDB_JOB_TYPE=smoke-test
  -e WANDB_MODE="${WANDB_MODE:-online}"
  -e WANDB_DISABLE_CODE=true
)
if [[ -f "$HOME/.netrc" ]]; then
  docker_args+=( -v "$HOME/.netrc:/root/.netrc:ro" )
fi

docker "${docker_args[@]}" "$image" \
  python /opt/mmdetection3d/tools/train.py \
  /workspace/AjouNice2026/perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py \
  --work-dir /workspace/AjouNice2026/artifacts/detection3d/bevfusion/training/base3_b0_smoke \
  --cfg-options \
    train_dataloader.dataset.ann_file=/workspace/AjouNice2026/artifacts/datasets/morai_base3_bevfusion_smoke_v1/morai_base3_infos_train.pkl \
    train_dataloader.batch_size="${SMOKE_BATCH_SIZE:-4}" \
    val_dataloader.dataset.ann_file=/workspace/AjouNice2026/artifacts/datasets/morai_base3_bevfusion_smoke_v1/morai_base3_infos_val.pkl \
    test_dataloader.dataset.ann_file=/workspace/AjouNice2026/artifacts/datasets/morai_base3_bevfusion_smoke_v1/morai_base3_infos_test.pkl \
    visualizer.vis_backends.1.init_kwargs.name="${WANDB_RUN_NAME:-bevfusion-base3-b0-smoke}" \
    visualizer.vis_backends.1.init_kwargs.job_type=smoke-test \
    train_cfg.max_epochs=1 \
    train_cfg.val_interval=1 \
    train_dataloader.num_workers=0 \
    train_dataloader.persistent_workers=false \
    val_dataloader.num_workers=0 \
    val_dataloader.persistent_workers=false \
    default_hooks.checkpoint.max_keep_ckpts=1
