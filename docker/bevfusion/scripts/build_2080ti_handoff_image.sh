#!/usr/bin/env bash
set -euo pipefail

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
image="${BEVFUSION_HANDOFF_IMAGE:-ajounice/bevfusion:cu128-sm75-handoff}"
base_image="${BEVFUSION_BASE_IMAGE:-ajounice/bevfusion:dev-cu128}"

docker build \
  --build-arg "BASE_IMAGE=$base_image" \
  --tag "$image" \
  --file "$repo_root/docker/bevfusion/Dockerfile.sm75-handoff" \
  "$repo_root"

echo "Built $image"
echo "Export with: docker save $image | gzip -1 > bevfusion-cu128-sm75-handoff.tar.gz"
