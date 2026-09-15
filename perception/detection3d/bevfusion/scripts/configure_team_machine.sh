#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: configure_team_machine.sh --operator {sw|yj|jh} [options]

Options:
  --capture-root PATH  Extracted Base3 capture root
  --info-root PATH     Portable Base3 BEVFusion metadata root
  --image IMAGE        Team Docker image tag
  --output-root PATH   Training output root
  --nuscenes-checkpoint PATH  MORAI-adapted nuScenes initialization
  --e10-full-checkpoint PATH  Full Base3 E10 checkpoint
  --inference-checkpoint PATH Inference-only Base3 E10 checkpoint
EOF
}

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
operator=""
capture_root="${BASE3_CAPTURE_ROOT:-}"
info_root="${BASE3_INFO_ROOT:-$repo_root/artifacts/datasets/morai_base3_bevfusion_portable_v1}"
image="${BEVFUSION_IMAGE:-ajounice/bevfusion:team-cu128-v1}"
output_root="${BEVFUSION_OUTPUT_ROOT:-$repo_root/artifacts/detection3d/bevfusion/team_training}"
nuscenes_checkpoint="${BEVFUSION_NUSCENES_ADAPTED_CHECKPOINT:-$repo_root/artifacts/detection3d/bevfusion/pretrained/bevfusion_nuscenes_morai3_compatible.pth}"
e10_full_checkpoint="${BEVFUSION_BASE3_E10_FULL_CHECKPOINT:-$repo_root/artifacts/detection3d/bevfusion/training/base3_b0_4cam_lidar_e12/epoch_10.pth}"
inference_checkpoint="${BEVFUSION_INFERENCE_CHECKPOINT:-$repo_root/artifacts/detection3d/bevfusion/releases/base3_bf0_e10/base3_bf0_e10_inference.pth}"

while (($#)); do
  case "$1" in
    --operator) operator="${2:?missing operator}"; shift 2 ;;
    --capture-root) capture_root="${2:?missing capture root}"; shift 2 ;;
    --info-root) info_root="${2:?missing info root}"; shift 2 ;;
    --image) image="${2:?missing image}"; shift 2 ;;
    --output-root) output_root="${2:?missing output root}"; shift 2 ;;
    --nuscenes-checkpoint) nuscenes_checkpoint="${2:?missing checkpoint}"; shift 2 ;;
    --e10-full-checkpoint) e10_full_checkpoint="${2:?missing checkpoint}"; shift 2 ;;
    --inference-checkpoint) inference_checkpoint="${2:?missing checkpoint}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$operator" in sw|yj|jh) ;; *) echo "--operator must be sw, yj, or jh" >&2; exit 2 ;; esac

if [[ -z "$capture_root" ]]; then
  local_capture="/home/luv/morai_cap/260829_034057_base3_static_10k"
  if [[ -d "$local_capture" ]]; then
    capture_root="$local_capture"
  else
    capture_root="/data/ajounice/base3-static-10k"
  fi
fi

env_file="$repo_root/.env.bevfusion.local"
{
  printf 'BEVFUSION_OPERATOR=%q\n' "$operator"
  printf 'BEVFUSION_IMAGE=%q\n' "$image"
  printf 'BASE3_CAPTURE_ROOT=%q\n' "$capture_root"
  printf 'BASE3_INFO_ROOT=%q\n' "$info_root"
  printf 'BEVFUSION_OUTPUT_ROOT=%q\n' "$output_root"
  printf 'BEVFUSION_NUSCENES_ADAPTED_CHECKPOINT=%q\n' "$nuscenes_checkpoint"
  printf 'BEVFUSION_BASE3_E10_FULL_CHECKPOINT=%q\n' "$e10_full_checkpoint"
  printf 'BEVFUSION_INFERENCE_CHECKPOINT=%q\n' "$inference_checkpoint"
  printf 'WANDB_ENTITY=%q\n' "AjouNice2026"
  printf 'WANDB_PROJECT=%q\n' "morai-3d-object-detection"
} >"$env_file"

experiment_root="$repo_root/perception/detection3d/bevfusion/configs/team/experiments/$operator"
mkdir -p "$experiment_root"

write_experiment() {
  local path="$1" name="$2" mode="$3" epochs="$4" lr="$5" tags="$6"
  [[ -e "$path" ]] && return
  sed \
    -e "s/name: fresh-e12/name: $name/" \
    -e "s/mode: fresh/mode: $mode/" \
    -e "s/epochs: 12/epochs: $epochs/" \
    -e "s/learning_rate: 1.0e-4/learning_rate: $lr/" \
    -e "s/tags: \[base3, four-camera, lidar, fresh\]/tags: [$tags]/" \
    "$repo_root/perception/detection3d/bevfusion/configs/team/experiments/experiment.example.yaml" \
    >"$path"
}

write_experiment "$experiment_root/fresh-e12.yaml" fresh-e12 fresh 12 1.0e-4 \
  "base3, four-camera, lidar, fresh"
write_experiment "$experiment_root/finetune-e6.yaml" finetune-e6 finetune 6 2.0e-5 \
  "base3, four-camera, lidar, finetune"
write_experiment "$experiment_root/random-e24.yaml" random-e24 random 24 1.0e-4 \
  "base3, four-camera, lidar, random-control"

echo "Created local machine config: $env_file"
echo "Created/kept local experiments: $experiment_root"
echo "Review paths, then dry-run with:"
echo "  bash perception/detection3d/bevfusion/scripts/run_team_training.sh fresh-e12 --dry-run"
