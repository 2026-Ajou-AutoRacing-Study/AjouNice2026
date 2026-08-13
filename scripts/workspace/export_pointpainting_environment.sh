#!/usr/bin/env bash
# Export a teammate's PointPainting/OpenPCDet environment into one shareable archive.

set -uo pipefail

CONDA_ENV="${CONDA_ENV:-openpcdet}"
OPENPCDET_ROOT="${OPENPCDET_ROOT:-$HOME/OpenPCDet}"
POINTPAINTING_CKPT="${POINTPAINTING_CKPT:-$OPENPCDET_ROOT/output/custom_models/pointpillar_morai_pointpainting/pointpainting_20epoch/ckpt/checkpoint_epoch_20.pth}"
OUTPUT_PARENT="${OUTPUT_PARENT:-$HOME/pointpainting_artifacts/environments}"

usage() {
  cat <<'EOF'
Usage:
  export_pointpainting_environment.sh [options]

Options:
  --conda-env NAME         Conda environment name (default: openpcdet)
  --openpcdet-root PATH    OpenPCDet checkout (default: ~/OpenPCDet)
  --checkpoint PATH        PointPainting checkpoint path
  --output-parent PATH     Directory that receives the export folder/archive
                           (default: ~/pointpainting_artifacts/environments)
  -h, --help               Show this help

The script does not copy the checkpoint. It records only its size and SHA256.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --conda-env)
      CONDA_ENV="$2"
      shift 2
      ;;
    --openpcdet-root)
      OPENPCDET_ROOT="$2"
      shift 2
      ;;
    --checkpoint)
      POINTPAINTING_CKPT="$2"
      shift 2
      ;;
    --output-parent)
      OUTPUT_PARENT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

OPENPCDET_ROOT="${OPENPCDET_ROOT/#\~/$HOME}"
POINTPAINTING_CKPT="${POINTPAINTING_CKPT/#\~/$HOME}"
OUTPUT_PARENT="${OUTPUT_PARENT/#\~/$HOME}"

mkdir -p "$OUTPUT_PARENT"

if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [[ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/anaconda3/etc/profile.d/conda.sh"
elif command -v conda >/dev/null 2>&1; then
  eval "$(conda shell.bash hook)"
else
  echo "[ERROR] Conda was not found." >&2
  exit 1
fi

if ! conda env list | awk 'NR > 2 {print $1}' | grep -Fxq "$CONDA_ENV"; then
  echo "[ERROR] Conda environment not found: $CONDA_ENV" >&2
  exit 1
fi

if [[ ! -d "$OPENPCDET_ROOT" ]]; then
  echo "[ERROR] OpenPCDet directory not found: $OPENPCDET_ROOT" >&2
  exit 1
fi

conda activate "$CONDA_ENV"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
EXPORT_NAME="pointpainting_environment_${TIMESTAMP}"
EXPORT_DIR="$OUTPUT_PARENT/$EXPORT_NAME"
ARCHIVE_PATH="$OUTPUT_PARENT/${EXPORT_NAME}.tar.gz"

mkdir -p "$EXPORT_DIR/configs"

echo "[INFO] Exporting Conda environment: $CONDA_ENV"
if ! conda env export --no-builds \
  | awk '!/^prefix: /' \
  > "$EXPORT_DIR/pointpainting_environment.yml"; then
  echo "[ERROR] conda env export failed." >&2
  exit 1
fi

conda list --explicit \
  > "$EXPORT_DIR/pointpainting_conda_explicit.txt" 2>&1 || true

python -m pip list --format=freeze \
  > "$EXPORT_DIR/pointpainting_pip_list.txt" 2>&1 || true

python -m pip freeze \
  > "$EXPORT_DIR/pointpainting_pip_freeze.txt" 2>&1 || true

{
  echo "exported_at=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "conda_environment=$CONDA_ENV"
  echo "conda_prefix=${CONDA_PREFIX:-}"
  echo "python_executable=$(command -v python || true)"
  echo "openpcdet_root=$OPENPCDET_ROOT"
  echo "checkpoint_path=$POINTPAINTING_CKPT"
} > "$EXPORT_DIR/export_metadata.txt"

{
  echo "===== OS ====="
  uname -a
  if [[ -f /etc/os-release ]]; then
    cat /etc/os-release
  fi
  echo
  echo "===== NVIDIA SMI ====="
  nvidia-smi 2>&1 || true
  echo
  echo "===== NVCC ====="
  nvcc --version 2>&1 || true
  echo
  echo "===== ROS ====="
  rosversion -d 2>&1 || true
  rosversion ros_comm 2>&1 || true
} > "$EXPORT_DIR/os_gpu_ros.txt"

OPENPCDET_ROOT="$OPENPCDET_ROOT" python - <<'PY' \
  > "$EXPORT_DIR/python_runtime.txt" 2>&1 || true
import importlib
import os
import platform
import sys

root = os.environ["OPENPCDET_ROOT"]
if root not in sys.path:
    sys.path.insert(0, root)

print("Python:", sys.version)
print("Executable:", sys.executable)
print("Platform:", platform.platform())

for name in [
    "numpy",
    "cv2",
    "torch",
    "torchvision",
    "spconv",
    "pcdet",
    "rospy",
    "cv_bridge",
]:
    try:
        module = importlib.import_module(name)
        version = getattr(module, "__version__", "<no __version__>")
        location = getattr(module, "__file__", "<no __file__>")
        print(f"{name}: OK version={version} file={location}")
    except Exception as exc:
        print(f"{name}: ERROR {type(exc).__name__}: {exc}")

try:
    import torch

    print("Torch CUDA:", torch.version.cuda)
    print("CUDA available:", torch.cuda.is_available())
    print("Torch arch list:", torch.cuda.get_arch_list())
    if torch.cuda.is_available():
        print("GPU:", torch.cuda.get_device_name(0))
        print("GPU capability:", torch.cuda.get_device_capability(0))
except Exception as exc:
    print("CUDA inspection error:", type(exc).__name__, exc)
PY

{
  echo "===== REMOTES ====="
  git -C "$OPENPCDET_ROOT" remote -v 2>&1 \
    | sed -E 's#(https?://)[^/@]+@#\1<redacted>@#g'
  echo
  echo "===== HEAD ====="
  git -C "$OPENPCDET_ROOT" rev-parse HEAD 2>&1
  git -C "$OPENPCDET_ROOT" log -1 --oneline --decorate 2>&1
  echo
  echo "===== SUBMODULES ====="
  git -C "$OPENPCDET_ROOT" submodule status --recursive 2>&1 || true
} > "$EXPORT_DIR/openpcdet_git.txt"

git -C "$OPENPCDET_ROOT" status --short --branch \
  > "$EXPORT_DIR/openpcdet_status.txt" 2>&1 || true

git -C "$OPENPCDET_ROOT" diff --binary \
  > "$EXPORT_DIR/openpcdet_uncommitted.patch" 2>&1 || true

git -C "$OPENPCDET_ROOT" diff --cached --binary \
  > "$EXPORT_DIR/openpcdet_staged.patch" 2>&1 || true

MODEL_CONFIG="$OPENPCDET_ROOT/tools/cfgs/custom_models/pointpillar_morai_pointpainting.yaml"
DATASET_CONFIG="$OPENPCDET_ROOT/tools/cfgs/dataset_configs/morai_pointpainting_dataset.yaml"

if [[ -f "$MODEL_CONFIG" ]]; then
  cp "$MODEL_CONFIG" "$EXPORT_DIR/configs/"
else
  echo "MISSING: $MODEL_CONFIG" \
    > "$EXPORT_DIR/configs/MISSING_model_config.txt"
fi

if [[ -f "$DATASET_CONFIG" ]]; then
  cp "$DATASET_CONFIG" "$EXPORT_DIR/configs/"
else
  echo "MISSING: $DATASET_CONFIG" \
    > "$EXPORT_DIR/configs/MISSING_dataset_config.txt"
fi

{
  if [[ -f "$POINTPAINTING_CKPT" ]]; then
    stat -c 'size_bytes=%s' "$POINTPAINTING_CKPT"
    sha256sum "$POINTPAINTING_CKPT"
  else
    echo "MISSING: $POINTPAINTING_CKPT"
  fi
} > "$EXPORT_DIR/checkpoint_sha256.txt"

cat > "$EXPORT_DIR/README.txt" <<EOF
PointPainting environment export
================================

Conda environment: $CONDA_ENV
OpenPCDet root:     $OPENPCDET_ROOT
Checkpoint:         $POINTPAINTING_CKPT

Important:
- The checkpoint itself is NOT included; only its size and SHA256 are recorded.
- Inspect openpcdet_status.txt and the two patch files for uncommitted work.
- Inspect pointpainting_environment.yml for private channels or credentials.
- OpenPCDet editable installs may need to be reinstalled after cloning the
  repository at the commit recorded in openpcdet_git.txt.
- CUDA extensions may need rebuilding for a different GPU architecture.
EOF

tar -czf "$ARCHIVE_PATH" \
  -C "$OUTPUT_PARENT" \
  "$EXPORT_NAME"

echo
echo "[OK] Export directory:"
echo "  $EXPORT_DIR"
echo "[OK] Share this archive:"
echo "  $ARCHIVE_PATH"
echo "[OK] Archive SHA256:"
sha256sum "$ARCHIVE_PATH"
echo
echo "[NOTICE] Review the archive for private paths or credentials before sharing."
