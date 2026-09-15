#!/usr/bin/env bash
set -euo pipefail

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
stamp="${HANDOFF_STAMP:-$(date +%Y%m%d_%H%M%S)}"
output_root="${HANDOFF_OUTPUT_ROOT:-$repo_root/artifacts/handoffs/bevfusion_2080ti_training_$stamp}"
support_stage="$output_root/support"
checkpoint_stage="$output_root/checkpoints"
image="${BEVFUSION_HANDOFF_IMAGE:-ajounice/bevfusion:cu128-sm75-handoff}"

metadata="$repo_root/artifacts/datasets/morai_base3_static_10k_pointpainting_input_v1"
infos="$repo_root/artifacts/datasets/morai_base3_bevfusion_portable_v1"
pretrained="$repo_root/artifacts/detection3d/bevfusion/pretrained/bevfusion_nuscenes_morai3_compatible.pth"
e10="$repo_root/artifacts/detection3d/bevfusion/training/base3_b0_4cam_lidar_e12/epoch_10.pth"

for path in "$metadata" "$infos" "$pretrained" "$e10"; do
  test -e "$path" || { echo "Missing required handoff input: $path" >&2; exit 2; }
done

# Staging is generated entirely by this script. Recreate it so repeated
# packaging never nests a previous copy inside itself.
rm -rf "$support_stage" "$checkpoint_stage"
mkdir -p "$support_stage" "$checkpoint_stage/artifacts/detection3d/bevfusion/pretrained" \
  "$checkpoint_stage/artifacts/detection3d/bevfusion/handoff"

tracked_files=(
  docker/bevfusion/Dockerfile
  docker/bevfusion/Dockerfile.sm75-handoff
  docker/bevfusion/scripts/build_2080ti_handoff_image.sh
  docker/bevfusion/scripts/verify_2080ti_handoff_image.sh
  docker/bevfusion/README.md
  docs/bevfusion/BASE3_BEVFUSION_2080TI_TRAINING_HANDOFF.md
  docs/bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md
  docs/bevfusion/BASE3_BF1_ONLINE_DOMAIN_FINETUNE.md
  perception/detection3d/bevfusion/README.md
  perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py
  perception/detection3d/bevfusion/scripts/prepare_base3_portable_infos.sh
  perception/detection3d/bevfusion/scripts/package_2080ti_training_handoff.sh
  perception/detection3d/bevfusion/scripts/run_base3_2080ti_smoke.sh
  perception/detection3d/bevfusion/scripts/run_base3_2080ti_training.sh
  perception/detection3d/bevfusion/tools/build_base3_bevfusion_infos.py
  perception/detection3d/bevfusion/tools/validate_base3_bevfusion_infos.py
  perception/detection3d/bevfusion/tools/test_portable_path_mapping.py
)
for relative in "${tracked_files[@]}"; do
  mkdir -p "$support_stage/$(dirname "$relative")"
  cp "$repo_root/$relative" "$support_stage/$relative"
done
cp "$repo_root/docs/bevfusion/BASE3_BEVFUSION_2080TI_TRAINING_HANDOFF.md" \
  "$support_stage/README_FIRST.md"
mkdir -p "$support_stage/artifacts/datasets"
cp -a "$metadata" \
  "$support_stage/artifacts/datasets/morai_base3_static_10k_pointpainting_input_v1"
cp -a "$infos" \
  "$support_stage/artifacts/datasets/morai_base3_bevfusion_portable_v1"

cp "$pretrained" \
  "$checkpoint_stage/artifacts/detection3d/bevfusion/pretrained/bevfusion_nuscenes_morai3_compatible.pth"
cp "$e10" \
  "$checkpoint_stage/artifacts/detection3d/bevfusion/handoff/base3_bf0_epoch10_full.pth"

(
  cd "$support_stage"
  find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
)
(
  cd "$checkpoint_stage"
  find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS
)

support_archive="$output_root/bevfusion_2080ti_support.tar.gz"
checkpoint_archive="$output_root/bevfusion_2080ti_checkpoints.tar.gz"
tar -C "$support_stage" -czf "$support_archive" .
tar -C "$checkpoint_stage" -czf "$checkpoint_archive" .

if [[ "${EXPORT_DOCKER_IMAGE:-0}" == 1 ]]; then
  docker image inspect "$image" >/dev/null
  image_archive="$output_root/bevfusion_cu128_sm75_handoff_image.tar.gz"
  # High compression lowers peak disk pressure while docker save materializes
  # its layer stream. This is slow but is a one-time transfer artifact.
  docker save "$image" | gzip -9 > "$image_archive"
fi

(cd "$output_root" && sha256sum ./*.tar.gz > ARCHIVE_SHA256SUMS)
if [[ "${KEEP_HANDOFF_STAGING:-0}" != 1 ]]; then
  rm -rf "$support_stage" "$checkpoint_stage"
fi
du -h "$output_root"/*.tar.gz
echo "Handoff output: $output_root"
