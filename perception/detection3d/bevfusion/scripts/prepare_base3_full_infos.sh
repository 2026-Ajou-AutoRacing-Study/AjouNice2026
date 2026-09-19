#!/usr/bin/env bash
set -euo pipefail

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
output_root="$repo_root/artifacts/datasets/morai_base3_bevfusion_v1"
python3 "$repo_root/perception/detection3d/bevfusion/tools/build_base3_bevfusion_infos.py" \
  --dataset-root "$repo_root/artifacts/datasets/morai_base3_static_10k_pointpainting_input_v1" \
  --output-dir "$output_root" \
  --minimum-gt-points 5 \
  --box-tolerance-m 0.03

python3 "$repo_root/perception/detection3d/bevfusion/tools/validate_base3_bevfusion_infos.py" \
  --info-dir "$output_root"
