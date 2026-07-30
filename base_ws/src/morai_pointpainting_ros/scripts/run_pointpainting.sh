#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-live}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CATKIN_WS="$(cd "$PACKAGE_DIR/../.." && pwd)"

CONDA_ENV="${CONDA_ENV:-openpcdet}"
OPENPCDET_ROOT="${OPENPCDET_ROOT:-$HOME/OpenPCDet}"

POINTPAINTING_CFG="${POINTPAINTING_CFG:-$PACKAGE_DIR/config/openpcdet/pointpillar_morai_pointpainting.yaml}"

POINTPAINTING_CKPT="${POINTPAINTING_CKPT:-$OPENPCDET_ROOT/output/custom_models/pointpillar_morai_pointpainting/pointpainting_20epoch/ckpt/checkpoint_epoch_20.pth}"
POINTPAINTING_REAL_INPUT_WARMUP="${POINTPAINTING_REAL_INPUT_WARMUP:-true}"
POINTPAINTING_REAL_INPUT_WARMUP_ITERATIONS="${POINTPAINTING_REAL_INPUT_WARMUP_ITERATIONS:-2}"

case "$MODE" in
    live)
        USE_SIM_TIME=false
        ;;
    bag)
        USE_SIM_TIME=true
        ;;
    *)
        echo "Usage: $0 [live|bag]"
        exit 1
        ;;
esac

# Conda 초기화
if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [[ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]]; then
    source "$HOME/anaconda3/etc/profile.d/conda.sh"
elif command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
else
    echo "[ERROR] Conda를 찾을 수 없습니다."
    exit 1
fi

# CUDA Toolkit Conda activation hooks may read optional variables such as
# NVCC_PREPEND_FLAGS before defining them. Temporarily disable nounset while
# Conda runs its activation hooks, then restore the script's strict mode.
set +u
conda activate "$CONDA_ENV"
set -u

source /opt/ros/noetic/setup.bash

if [[ ! -f "$CATKIN_WS/devel/setup.bash" ]]; then
    echo "[ERROR] Catkin workspace가 빌드되지 않았습니다."
    echo "실행:"
    echo "  cd $CATKIN_WS"
    echo '  catkin_make -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python"'
    exit 1
fi

source "$CATKIN_WS/devel/setup.bash"

# 필수 경로 검사
if [[ ! -d "$OPENPCDET_ROOT" ]]; then
    echo "[ERROR] OpenPCDet 경로가 없습니다:"
    echo "  $OPENPCDET_ROOT"
    echo '  export OPENPCDET_ROOT="$HOME/OpenPCDet"'
    exit 1
fi

if [[ ! -f "$POINTPAINTING_CFG" ]]; then
    echo "[ERROR] PointPillars config가 없습니다:"
    echo "  $POINTPAINTING_CFG"
    echo '  export POINTPAINTING_CFG="/path/to/pointpillar_morai_pointpainting.yaml"'
    exit 1
fi

if [[ ! -f "$POINTPAINTING_CKPT" ]]; then
    echo "[ERROR] PointPillars checkpoint가 없습니다:"
    echo "  $POINTPAINTING_CKPT"
    echo '  export POINTPAINTING_CKPT="/path/to/checkpoint_epoch_20.pth"'
    exit 1
fi

# Conda libffi 충돌 방지
SYSTEM_LIBFFI="$(ldconfig -p | awk '/libffi\.so\.7/{print $NF; exit}')"

if [[ -n "$SYSTEM_LIBFFI" ]]; then
    export LD_PRELOAD="$SYSTEM_LIBFFI${LD_PRELOAD:+:$LD_PRELOAD}"
fi

rosparam set /use_sim_time "$USE_SIM_TIME"

echo "=============================================="
echo "PointPainting ROS"
echo "=============================================="
echo "Mode              : $MODE"
echo "Catkin workspace  : $CATKIN_WS"
echo "Conda environment : $CONDA_ENV"
echo "Python            : $(which python)"
echo "OpenPCDet         : $OPENPCDET_ROOT"
echo "Config            : $POINTPAINTING_CFG"
echo "Checkpoint        : $POINTPAINTING_CKPT"
echo "Real input warm-up: $POINTPAINTING_REAL_INPUT_WARMUP"
echo "Warm-up iterations: $POINTPAINTING_REAL_INPUT_WARMUP_ITERATIONS"
echo "use_sim_time      : $USE_SIM_TIME"
echo "=============================================="

exec roslaunch morai_pointpainting_ros \
    pointpainting_online.launch \
    python_executable:="$(command -v python)" \
    openpcdet_root:="$OPENPCDET_ROOT" \
    cfg_file:="$POINTPAINTING_CFG" \
    checkpoint:="$POINTPAINTING_CKPT" \
    publish_painted_points:=false \
    segmentation_short_side:=320 \
    warmup_segmentation:=true \
    warmup_full_pipeline:=true \
    warmup_iterations:=2 \
    warmup_point_count:=12000 \
    real_input_warmup:="$POINTPAINTING_REAL_INPUT_WARMUP" \
    real_input_warmup_iterations:="$POINTPAINTING_REAL_INPUT_WARMUP_ITERATIONS"
