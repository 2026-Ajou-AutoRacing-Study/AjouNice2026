#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 {fresh|finetune|random}"
  echo "  fresh:    new run from the adapted nuScenes initialization (recommended)"
  echo "  finetune: new low-LR run initialized from full Base3 E10 checkpoint"
  echo "  random:   strict random initialization (research control only)"
}

mode="${1:-}"
case "$mode" in fresh|finetune|random) ;; *) usage >&2; exit 2 ;; esac

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
image="${BEVFUSION_IMAGE:-ajounice/bevfusion:cu128-sm75-handoff}"
capture_root="${BASE3_CAPTURE_ROOT:?Set BASE3_CAPTURE_ROOT to the extracted Base3 capture root}"
info_root="${BASE3_INFO_ROOT:-$repo_root/artifacts/datasets/morai_base3_bevfusion_portable_v1}"
physical_batch="${BEVFUSION_BATCH_SIZE:-2}"
accumulation="${BEVFUSION_ACCUMULATION:-2}"
workers="${BEVFUSION_WORKERS:-3}"

test -d "$capture_root" || {
  echo "BASE3_CAPTURE_ROOT is not a directory: $capture_root" >&2
  exit 2
}

if [[ "$physical_batch" != 1 && "$physical_batch" != 2 ]]; then
  echo "RTX 2080 Ti handoff supports physical batch 1 or 2; got $physical_batch" >&2
  exit 2
fi
if [[ $((physical_batch * accumulation)) -ne 4 ]]; then
  echo "physical batch * accumulation must remain 4; got $physical_batch * $accumulation" >&2
  exit 2
fi
for split in train val test; do
  test -s "$info_root/morai_base3_infos_${split}.pkl" || {
    echo "Missing portable ${split} info. Run prepare_base3_portable_infos.sh." >&2
    exit 2
  }
done

pretrained="$repo_root/artifacts/detection3d/bevfusion/pretrained/bevfusion_nuscenes_morai3_compatible.pth"
e10_full="${BEVFUSION_FINETUNE_CHECKPOINT:-$repo_root/artifacts/detection3d/bevfusion/handoff/base3_bf0_epoch10_full.pth}"
case "$mode" in
  fresh)
    checkpoint="$pretrained"; epochs="${BEVFUSION_EPOCHS:-12}"; lr="${BEVFUSION_LR:-1e-4}"
    run_suffix="fresh-nuscenes-init-e${epochs}" ;;
  finetune)
    checkpoint="$e10_full"; epochs="${BEVFUSION_EPOCHS:-6}"; lr="${BEVFUSION_LR:-2e-5}"
    run_suffix="e10-finetune-e${epochs}" ;;
  random)
    checkpoint=""; epochs="${BEVFUSION_EPOCHS:-24}"; lr="${BEVFUSION_LR:-1e-4}"
    run_suffix="strict-random-e${epochs}" ;;
esac
if [[ -n "$checkpoint" && ! -s "$checkpoint" ]]; then
  echo "Missing initialization checkpoint: $checkpoint" >&2
  exit 2
fi

work_dir="${BEVFUSION_WORK_DIR:-$repo_root/artifacts/detection3d/bevfusion/training/2080ti_${run_suffix}}"
run_name="${WANDB_RUN_NAME:-bevfusion-base3-2080ti-${run_suffix}}"
mkdir -p "$work_dir"

warmup=$((500 * 4 / physical_batch))
container_work_dir="/outputs"
container_info_root="/datasets/infos"
cfg=(
  train_dataloader.batch_size="$physical_batch"
  train_dataloader.num_workers="$workers"
  optim_wrapper.accumulative_counts="$accumulation"
  optim_wrapper.optimizer.lr="$lr"
  param_scheduler.0.end="$warmup"
  train_cfg.max_epochs="$epochs"
  param_scheduler.1.end="$epochs"
  param_scheduler.1.T_max="$epochs"
  train_dataloader.dataset.ann_file="$container_info_root/morai_base3_infos_train.pkl"
  val_dataloader.dataset.ann_file="$container_info_root/morai_base3_infos_val.pkl"
  test_dataloader.dataset.ann_file="$container_info_root/morai_base3_infos_test.pkl"
  visualizer.vis_backends.1.init_kwargs.name="$run_name"
)
if [[ -n "$checkpoint" ]]; then
  cfg+=(load_from=/checkpoints/init.pth)
else
  cfg+=(load_from=None)
fi

docker_args=(
  run --rm --gpus all --shm-size="${BEVFUSION_SHM_SIZE:-10g}"
  --name "ajounice-bevfusion-${mode}-train"
  -v "$repo_root:/workspace/AjouNice2026"
  -v "$capture_root:/datasets/base3:ro"
  -v "$info_root:/datasets/infos:ro"
  -v "$work_dir:/outputs"
  -w /workspace/AjouNice2026/perception/detection3d/bevfusion
  -e PYTHONPATH=/workspace/AjouNice2026:/opt/mmdetection3d
  -e TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
  -e WANDB_MODE="${WANDB_MODE:-online}"
  -e WANDB_DISABLE_CODE=true
)
if [[ -n "$checkpoint" ]]; then
  docker_args+=( -v "$checkpoint:/checkpoints/init.pth:ro" )
fi
if [[ -f "$HOME/.netrc" ]]; then
  docker_args+=( -v "$HOME/.netrc:/root/.netrc:ro" )
fi
if [[ -n "${WANDB_ENTITY:-}" ]]; then
  cfg+=(visualizer.vis_backends.1.init_kwargs.entity="$WANDB_ENTITY")
fi
if [[ -n "${WANDB_PROJECT:-}" ]]; then
  cfg+=(visualizer.vis_backends.1.init_kwargs.project="$WANDB_PROJECT")
fi

printf 'mode=%s physical_batch=%s accumulation=%s effective_batch=4 lr=%s epochs=%s\n' \
  "$mode" "$physical_batch" "$accumulation" "$lr" "$epochs"
docker "${docker_args[@]}" "$image" \
  python /opt/mmdetection3d/tools/train.py \
  /workspace/AjouNice2026/perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py \
  --work-dir "$container_work_dir" --cfg-options "${cfg[@]}"
