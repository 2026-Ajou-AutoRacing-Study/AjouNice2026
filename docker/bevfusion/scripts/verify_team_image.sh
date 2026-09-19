#!/usr/bin/env bash
set -euo pipefail

image="${BEVFUSION_IMAGE:-ajounice/bevfusion:team-cu128-v1}"

docker image inspect "$image" >/dev/null
arch_list="$(docker run --rm --entrypoint bash "$image" -lc \
  'printf "%s\n" "$TORCH_CUDA_ARCH_LIST"')"

for arch in 7.5 8.6 8.9 12.0+PTX; do
  case ";$arch_list;" in
    *";$arch;"*) ;;
    *) echo "FAIL: image does not declare CUDA architecture $arch: $arch_list" >&2; exit 1 ;;
  esac
done

docker run --rm --entrypoint python "$image" -c '
import mmcv
import mmdet3d
import torch
import projects.BEVFusion.bevfusion
print("imports=ok")
print("torch=" + torch.__version__)
print("torch_cuda=" + str(torch.version.cuda))
'

# Verify every supported GPU family has native cubins. RTX 50 additionally
# retains PTX so a newer driver can JIT the code if required.
docker run --rm --entrypoint bash "$image" -lc '
set -e
found=0
for library in \
  /opt/mmcv/mmcv/_ext*.so \
  /opt/mmdetection3d/projects/BEVFusion/bevfusion/ops/bev_pool/*ext*.so \
  /opt/mmdetection3d/projects/BEVFusion/bevfusion/ops/voxel/*.so; do
  test -e "$library" || continue
  found=$((found + 1))
  elf="$(cuobjdump --list-elf "$library")"
  for arch in sm_75 sm_86 sm_89 sm_120; do
    printf "%s\n" "$elf" | grep -q "$arch" || {
      echo "missing $arch cubin: $library" >&2
      exit 1
    }
  done
  cuobjdump --list-ptx "$library" | grep -q sm_120 || {
    echo "missing sm_120 PTX: $library" >&2
    exit 1
  }
done
test "$found" -ge 3
'

docker run --rm --gpus all --entrypoint python "$image" -c '
import torch
assert torch.cuda.is_available(), "CUDA is unavailable in the container"
x = torch.ones((1024, 1024), device="cuda", dtype=torch.float16)
y = x @ x
torch.cuda.synchronize()
print("gpu=" + torch.cuda.get_device_name(0))
print("capability={}.{}".format(*torch.cuda.get_device_capability(0)))
print("fp16_cuda=ok sum={:.1f}".format(y[0, 0].item()))
'

echo "PASS: image=$image TORCH_CUDA_ARCH_LIST=$arch_list"
