#!/usr/bin/env bash
set -euo pipefail

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
image="${BEVFUSION_IMAGE:-ajounice/bevfusion:dev-cu128}"
info_root="$repo_root/artifacts/datasets/morai_base3_bevfusion_v1"
source_checkpoint="${BEVFUSION_SOURCE_CHECKPOINT:-$repo_root/artifacts/detection3d/bevfusion/training/base3_b0_4cam_lidar_e12/best_val_center_mAP_epoch_6.pth}"
work_dir="${BEVFUSION_WORK_DIR:-$repo_root/artifacts/detection3d/bevfusion/training/base3_bf1_online_domain_e6ft6}"
run_name="${WANDB_RUN_NAME:-bevfusion-base3-bf1-online-domain-e6ft6}"

for split in train val test; do
  test -s "$info_root/morai_base3_infos_${split}.pkl" || {
    echo "Missing full info for ${split}; run prepare_base3_full_infos.sh first." >&2
    exit 2
  }
done
test -s "$source_checkpoint" || {
  echo "Missing B3-BF0 E6 source checkpoint: $source_checkpoint" >&2
  exit 2
}
mkdir -p "$work_dir"

docker_args=(
  run --rm --gpus all --shm-size=12g
  --name ajounice-bevfusion-base3-bf1-train
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
container_checkpoint="/workspace/AjouNice2026/${source_checkpoint#"$repo_root/"}"
docker "${docker_args[@]}" "$image" \
  python /opt/mmdetection3d/tools/train.py \
  /workspace/AjouNice2026/perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py \
  --work-dir "$container_work_dir" \
  --cfg-options \
    load_from="$container_checkpoint" \
    train_cfg.max_epochs=6 \
    optim_wrapper.optimizer.lr=2e-5 \
    param_scheduler.1.end=6 \
    param_scheduler.1.T_max=6 \
    train_pipeline.4.jpeg_prob=0.8 \
    train_pipeline.4.jpeg_quality="[72,96]" \
    train_pipeline.4.blur_prob=0.12 \
    train_pipeline.4.exposure_prob=0.5 \
    train_pipeline.4.gain="[0.92,1.08]" \
    train_pipeline.4.bias="[-6.0,6.0]" \
    train_pipeline.4.noise_prob=0.2 \
    train_pipeline.4.noise_std="[0.0,1.5]" \
    train_dataloader.dataset.pipeline.4.jpeg_prob=0.8 \
    train_dataloader.dataset.pipeline.4.jpeg_quality="[72,96]" \
    train_dataloader.dataset.pipeline.4.blur_prob=0.12 \
    train_dataloader.dataset.pipeline.4.exposure_prob=0.5 \
    train_dataloader.dataset.pipeline.4.gain="[0.92,1.08]" \
    train_dataloader.dataset.pipeline.4.bias="[-6.0,6.0]" \
    train_dataloader.dataset.pipeline.4.noise_prob=0.2 \
    train_dataloader.dataset.pipeline.4.noise_std="[0.0,1.5]" \
    default_hooks.checkpoint.max_keep_ckpts=6 \
    visualizer.vis_backends.1.init_kwargs.name="$run_name"
