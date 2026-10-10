#!/usr/bin/env bash
# Vast/container-internal installer. Does NOT invoke Docker or rent anything.
# Use the PyTorch2.7.1/CUDA12.8 devel image, or the already installed team image.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ "${1:-}" != "--install" && "${1:-}" != "--check-only" ]]; then
  echo 'Usage: bash docker/bevfusion/scripts/setup_direct_environment.sh --check-only|--install'
  exit 2
fi
python - <<'PY'
import torch
assert torch.__version__.split('+')[0] == '2.7.1', 'Requires pinned torch2.7.1; do not mutate an unknown template silently'
assert torch.version.cuda == '12.8', 'Requires CUDA12.8 PyTorch environment'
assert torch.cuda.is_available(), 'GPU/driver not visible'
print('GPU:', torch.cuda.get_device_name(0), 'CUDA:', torch.version.cuda)
PY
if [[ "$1" == "--install" ]]; then
  command -v nvcc >/dev/null || { echo 'CUDA devel/nvcc required (runtime-only image is insufficient)'; exit 1; }
  if [[ "$(id -u)" == 0 ]]; then
    apt-get update
    apt-get install -y --no-install-recommends build-essential git libgl1 libglib2.0-0 ca-certificates
  else
    command -v g++ >/dev/null && command -v git >/dev/null || { echo 'Install build-essential/git as root first'; exit 1; }
    python -c 'import ctypes; ctypes.CDLL("libGL.so.1"); ctypes.CDLL("libglib-2.0.so.0")'
  fi
  export MAX_JOBS="${MAX_JOBS:-8}"
  export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.9}"
  python -m pip install -r "$ROOT/docker/bevfusion/requirements/base.txt" \
    -r "$ROOT/docker/bevfusion/requirements/inference.txt" \
    -r "$ROOT/docker/bevfusion/requirements/training.txt"
  MMCV_DIR="${MMCV_SOURCE_DIR:-$HOME/ajou_dependencies/mmcv-v2.1.0}"
  if [[ ! -d "$MMCV_DIR" ]]; then
    mkdir -p "$(dirname "$MMCV_DIR")"
    git clone --branch v2.1.0 --depth 1 https://github.com/open-mmlab/mmcv.git "$MMCV_DIR"
  fi
  [[ "$(git -C "$MMCV_DIR" describe --tags --exact-match)" == v2.1.0 ]] || { echo 'MMCV source tag differs'; exit 1; }
  FORCE_CUDA=1 MMCV_WITH_OPS=1 python -m pip install --verbose --editable "$MMCV_DIR" --no-build-isolation
  python -m pip install --editable "$ROOT/third_party/mmdetection3d" --no-deps --no-build-isolation
  (
    cd "$ROOT/third_party/mmdetection3d"
    FORCE_CUDA=1 python projects/BEVFusion/setup.py develop
  )
fi
# Respect the installed fork's extension location. The team image has built
# ops under /opt; an unbuilt source checkout must not shadow those modules.
MMDET_ROOT="$(python -c 'from pathlib import Path; import mmdet3d; print(Path(mmdet3d.__file__).resolve().parents[1])')"
export PYTHONPATH="$ROOT:$MMDET_ROOT${PYTHONPATH:+:$PYTHONPATH}"
python - <<'PY'
import torch, mmcv, mmengine, mmdet, mmdet3d, spconv, cv2
from mmcv.ops import nms_rotated
from projects.BEVFusion.bevfusion.ops import bev_pool, Voxelization
print('torch/mmcv/mmengine:', torch.__version__, mmcv.__version__, mmengine.__version__)
boxes=torch.tensor([[0.,0.,2.,1.,0.]],device='cuda')
scores=torch.ones(1,device='cuda')
assert nms_rotated(boxes,scores,.5)[1].numel()==1
print('CUDA rotated NMS and BEVFusion imports PASS; full separated-pool/model smoke follows separately')
PY
