#!/usr/bin/env bash
set -euo pipefail

image="${BEVFUSION_HANDOFF_IMAGE:-ajounice/bevfusion:cu128-sm75-handoff}"

docker image inspect "$image" >/dev/null
arch_list="$({ docker run --rm --entrypoint bash "$image" -lc \
  'printf "%s\n" "$TORCH_CUDA_ARCH_LIST"'; } 2>/dev/null)"
case ";$arch_list;" in
  *';7.5;'*) ;;
  *) echo "FAIL: $image does not declare CUDA architecture 7.5: $arch_list" >&2; exit 1 ;;
esac

docker run --rm --entrypoint python "$image" -c \
  'import mmcv, mmdet3d, torch; import projects.BEVFusion.bevfusion; print("imports=ok torch=" + torch.__version__)'
docker run --rm --entrypoint bash "$image" -lc '
set -e
for library in \
  /opt/mmcv/mmcv/_ext*.so \
  /opt/mmdetection3d/projects/BEVFusion/bevfusion/ops/bev_pool/*ext*.so \
  /opt/mmdetection3d/projects/BEVFusion/bevfusion/ops/voxel/*.so; do
  cuobjdump --list-elf "$library" | grep -q sm_75 || {
    echo "missing sm_75 cubin: $library" >&2
    exit 1
  }
done'
echo "PASS: image=$image TORCH_CUDA_ARCH_LIST=$arch_list"
