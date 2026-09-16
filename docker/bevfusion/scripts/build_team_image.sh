#!/usr/bin/env bash
set -euo pipefail

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
image="${BEVFUSION_IMAGE:-ajounice/bevfusion:team-cu128-v1}"
arch_list="${BEVFUSION_CUDA_ARCH_LIST:-7.5;8.6;8.9;12.0+PTX}"

docker build \
  --build-arg "TORCH_CUDA_ARCH_LIST=$arch_list" \
  --tag "$image" \
  --file "$repo_root/docker/bevfusion/Dockerfile" \
  "$repo_root"

echo "Built: $image"
echo "CUDA architectures: $arch_list"
