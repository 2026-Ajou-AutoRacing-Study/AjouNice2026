#!/usr/bin/env bash
set -euo pipefail

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
capture_root="${BASE3_CAPTURE_ROOT:?Set BASE3_CAPTURE_ROOT to the extracted Base3 capture root}"
metadata_root="${BASE3_METADATA_ROOT:-$repo_root/artifacts/datasets/morai_base3_static_10k_pointpainting_input_v1}"
output_root="${BASE3_INFO_ROOT:-$repo_root/artifacts/datasets/morai_base3_bevfusion_portable_v1}"
container_capture_root="${BASE3_CONTAINER_CAPTURE_ROOT:-/datasets/base3}"

test -d "$capture_root" || {
  echo "BASE3_CAPTURE_ROOT is not a directory: $capture_root" >&2
  exit 2
}
test -s "$metadata_root/metadata/dataset.json" || {
  echo "Missing Base3 metadata index: $metadata_root" >&2
  exit 2
}

python3 "$repo_root/perception/detection3d/bevfusion/tools/build_base3_bevfusion_infos.py" \
  --dataset-root "$metadata_root" \
  --output-dir "$output_root" \
  --capture-root "$capture_root" \
  --serialized-capture-root "$container_capture_root" \
  --minimum-gt-points 5 \
  --box-tolerance-m 0.03

python3 "$repo_root/perception/detection3d/bevfusion/tools/validate_base3_bevfusion_infos.py" \
  --info-dir "$output_root" \
  --capture-root "$capture_root" \
  --serialized-capture-root "$container_capture_root"
