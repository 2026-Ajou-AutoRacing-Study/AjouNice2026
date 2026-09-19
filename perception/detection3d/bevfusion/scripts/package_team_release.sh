#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: package_team_release.sh [--output-root PATH] [--skip-image] [--skip-dataset]

Creates independent .tar.zst archives for the shared Docker image,
checkpoints, full Base3 dataset, and a small smoke-test subset.
EOF
}

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
if [[ -f "$repo_root/.env.bevfusion.local" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$repo_root/.env.bevfusion.local"
  set +a
fi

stamp="$(date +%Y%m%d_%H%M%S)"
output_root="$repo_root/artifacts/handoffs/bevfusion_team_v1_$stamp"
skip_image=0
skip_dataset=0
while (($#)); do
  case "$1" in
    --output-root) output_root="${2:?missing output root}"; shift 2 ;;
    --skip-image) skip_image=1; shift ;;
    --skip-dataset) skip_dataset=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

command -v zstd >/dev/null || {
  echo "zstd is required: sudo apt update && sudo apt install -y zstd" >&2
  exit 2
}
[[ ! -e "$output_root" ]] || {
  echo "Refusing to overwrite release directory: $output_root" >&2
  exit 2
}
mkdir -p "$output_root"
staging="$output_root/.staging"
mkdir -p "$staging"
cleanup() { rm -rf -- "$staging"; }
trap cleanup EXIT

capture_root="${BASE3_CAPTURE_ROOT:?BASE3_CAPTURE_ROOT is required}"
info_root="${BASE3_INFO_ROOT:?BASE3_INFO_ROOT is required}"
image="${BEVFUSION_IMAGE:-ajounice/bevfusion:team-cu128-v1}"
nuscenes="${BEVFUSION_NUSCENES_ADAPTED_CHECKPOINT:?missing adapted checkpoint path}"
e10_full="${BEVFUSION_BASE3_E10_FULL_CHECKPOINT:?missing E10 full checkpoint path}"
production_inference="${BEVFUSION_INFERENCE_CHECKPOINT:?missing production inference checkpoint path}"

for path in "$capture_root" "$info_root" "$nuscenes" "$e10_full" "$production_inference"; do
  [[ -e "$path" ]] || { echo "Missing release input: $path" >&2; exit 2; }
done

checkpoint_stage="$staging/bevfusion-team-checkpoints-v1"
mkdir -p "$checkpoint_stage/checkpoints"
cp --reflink=auto "$nuscenes" "$checkpoint_stage/checkpoints/bevfusion_nuscenes_morai3_compatible.pth"
cp --reflink=auto "$e10_full" "$checkpoint_stage/checkpoints/base3_bf0_epoch10_full.pth"
cp --reflink=auto "$production_inference" "$checkpoint_stage/checkpoints/base3_bf0_f40_e25_inference.pth"
cat >"$checkpoint_stage/README.txt" <<'EOF'
bevfusion_nuscenes_morai3_compatible.pth: fresh Base3 initialization
base3_bf0_epoch10_full.pth: finetune or exact compatible resume checkpoint
base3_bf0_f40_e25_inference.pth: production inference-only checkpoint
EOF
(
  cd "$checkpoint_stage"
  find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum >SHA256SUMS
)
tar -C "$staging" -cf - "$(basename "$checkpoint_stage")" \
  | zstd -T0 -10 -o "$output_root/bevfusion-team-checkpoints-v1.tar.zst"

smoke_stage="$staging/base3-smoke-subset-v1"
python3 "$repo_root/perception/detection3d/bevfusion/tools/build_team_smoke_bundle.py" \
  --info-root "$info_root" \
  --capture-root "$capture_root" \
  --output-root "$smoke_stage" \
  --samples-per-split 16 \
  --link-files
tar -C "$staging" -cf - "$(basename "$smoke_stage")" \
  | zstd -T0 -6 -o "$output_root/base3-smoke-subset-v1.tar.zst"

if [[ "$skip_dataset" == 0 ]]; then
  dataset_stage="$staging/base3-static-10k-v1"
  python3 "$repo_root/perception/detection3d/bevfusion/tools/build_team_smoke_bundle.py" \
    --info-root "$info_root" \
    --capture-root "$capture_root" \
    --output-root "$dataset_stage" \
    --samples-per-split 0 \
    --symlink-files
  metadata="$repo_root/artifacts/datasets/morai_base3_static_10k_pointpainting_input_v1"
  if [[ -d "$metadata" ]]; then
    cp -a "$metadata" "$dataset_stage/base3_metadata"
  fi
  tar --dereference -C "$staging" -cf - "$(basename "$dataset_stage")" \
    | zstd -T0 -6 -o "$output_root/base3-static-10k-v1.tar.zst"
fi

if [[ "$skip_image" == 0 ]]; then
  docker image inspect "$image" >/dev/null
  docker save "$image" \
    | zstd -T0 -10 -o "$output_root/ajounice-bevfusion-team-cu128-v1.image.tar.zst"
fi

cp "$repo_root/docs/bevfusion/TEAM_DOCKER_TRAINING_HANDOFF.md" "$output_root/README_FIRST.md"
(
  cd "$output_root"
  find . -maxdepth 1 -type f -name '*.tar.zst' -print0 \
    | sort -z | xargs -0 sha256sum >SHA256SUMS
)
{
  echo "created_at=$stamp"
  echo "git_sha=$(git -C "$repo_root" rev-parse HEAD)"
  echo "image=$image"
  echo "capture_root=$capture_root"
  echo "info_root=$info_root"
  echo "archives:"
  du -h "$output_root"/*.tar.zst
} >"$output_root/ARTIFACT_MANIFEST.txt"

echo "Team release: $output_root"
cat "$output_root/ARTIFACT_MANIFEST.txt"
