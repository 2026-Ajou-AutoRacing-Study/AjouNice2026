#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-live}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CATKIN_WS="$(cd "$PACKAGE_DIR/../.." && pwd)"

CONDA_ENV="${CONDA_ENV:-openpcdet_team}"
OPENPCDET_ROOT="${OPENPCDET_ROOT:-$HOME/OpenPCDet}"
REPO_ROOT="$(cd "$CATKIN_WS/.." && pwd)"

POINTPAINTING_RELEASE="${POINTPAINTING_RELEASE:-$REPO_ROOT/artifacts/detection3d/pointpainting/releases/260817_sunny13_1500_9feat_80e}"
POINTPAINTING_CFG="${POINTPAINTING_CFG:-$REPO_ROOT/perception/pointpainting/openpcdet_patch/tools/cfgs/custom_models/pointpillar_morai_pointpainting_sunny13_1500_9feat.yaml}"

POINTPAINTING_CKPT="${POINTPAINTING_CKPT:-$POINTPAINTING_RELEASE/pointpainting_B0_best_epoch65.pth}"
POINTPAINTING_SEG_CKPT="${POINTPAINTING_SEG_CKPT:-$POINTPAINTING_RELEASE/lraspp_4class_best_miou.pth}"
POINTPAINTING_REAL_INPUT_WARMUP="${POINTPAINTING_REAL_INPUT_WARMUP:-true}"
POINTPAINTING_REAL_INPUT_WARMUP_ITERATIONS="${POINTPAINTING_REAL_INPUT_WARMUP_ITERATIONS:-2}"
POINTPAINTING_WITH_TP="${POINTPAINTING_WITH_TP:-false}"
POINTPAINTING_PREDICTION_MODEL="${POINTPAINTING_PREDICTION_MODEL:-map_based}"
POINTPAINTING_BENCHMARK_DIR="${POINTPAINTING_BENCHMARK_DIR:-}"
POINTPAINTING_BENCHMARK_LABEL="${POINTPAINTING_BENCHMARK_LABEL:-pointpainting}"
POINTPAINTING_BENCHMARK_EXCLUDE_FRAMES="${POINTPAINTING_BENCHMARK_EXCLUDE_FRAMES:-10}"
POINTPAINTING_DETAILED_TIMING="${POINTPAINTING_DETAILED_TIMING:-false}"
POINTPAINTING_PREPROCESS_BACKEND="${POINTPAINTING_PREPROCESS_BACKEND:-torchvision}"
POINTPAINTING_CAMERA_CPU_WORKERS="${POINTPAINTING_CAMERA_CPU_WORKERS:-4}"

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

# PyTorch 2.6+ defaults torch.load() to weights_only=True, while the trusted
# team OpenPCDet checkpoint contains legacy NumPy metadata.
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1

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

if [[ ! -f "$POINTPAINTING_SEG_CKPT" ]]; then
    echo "[ERROR] LRASPP checkpoint가 없습니다:"
    echo "  $POINTPAINTING_SEG_CKPT"
    exit 1
fi

# Conda libffi 충돌 방지
# ``awk ... exit`` may close the pipe before ``ldconfig`` finishes. With
# ``pipefail`` that harmless SIGPIPE used to terminate redirected/batch runs
# before the script printed any diagnostics.
SYSTEM_LIBFFI="$(ldconfig -p 2>/dev/null | awk '/libffi\.so\.7/{print $NF; exit}' || true)"

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
echo "Seg checkpoint    : $POINTPAINTING_SEG_CKPT"
echo "Real input warm-up: $POINTPAINTING_REAL_INPUT_WARMUP"
echo "Warm-up iterations: $POINTPAINTING_REAL_INPUT_WARMUP_ITERATIONS"
echo "With T&P          : $POINTPAINTING_WITH_TP"
echo "Prediction model  : $POINTPAINTING_PREDICTION_MODEL"
echo "Benchmark dir     : ${POINTPAINTING_BENCHMARK_DIR:-disabled}"
echo "Detailed timing   : $POINTPAINTING_DETAILED_TIMING"
echo "Preprocess backend: $POINTPAINTING_PREPROCESS_BACKEND"
echo "Camera CPU workers: $POINTPAINTING_CAMERA_CPU_WORKERS"
echo "use_sim_time      : $USE_SIM_TIME"
echo "=============================================="

if [[ "$POINTPAINTING_WITH_TP" == "true" ]]; then
    LAUNCH_FILE=pointpainting_with_tp.launch
    EXTRA_LAUNCH_ARGS=(
        prediction_model:="$POINTPAINTING_PREDICTION_MODEL"
    )
else
    LAUNCH_FILE=pointpainting_online.launch
    EXTRA_LAUNCH_ARGS=()
fi

exec roslaunch morai_pointpainting_ros \
    "$LAUNCH_FILE" \
    use_sim_time:="$USE_SIM_TIME" \
    python_executable:="$(command -v python)" \
    openpcdet_root:="$OPENPCDET_ROOT" \
    repo_root:="$REPO_ROOT" \
    cfg_file:="$POINTPAINTING_CFG" \
    checkpoint:="$POINTPAINTING_CKPT" \
    segmentation_checkpoint:="$POINTPAINTING_SEG_CKPT" \
    publish_painted_points:=false \
    segmentation_short_side:=384 \
    segmentation_preprocess_backend:="$POINTPAINTING_PREPROCESS_BACKEND" \
    camera_cpu_workers:="$POINTPAINTING_CAMERA_CPU_WORKERS" \
    warmup_segmentation:=true \
    warmup_full_pipeline:=true \
    warmup_iterations:=2 \
    warmup_point_count:=12000 \
    real_input_warmup:="$POINTPAINTING_REAL_INPUT_WARMUP" \
    real_input_warmup_iterations:="$POINTPAINTING_REAL_INPUT_WARMUP_ITERATIONS" \
    benchmark_output_dir:="$POINTPAINTING_BENCHMARK_DIR" \
    benchmark_label:="$POINTPAINTING_BENCHMARK_LABEL" \
    benchmark_exclude_frames:="$POINTPAINTING_BENCHMARK_EXCLUDE_FRAMES" \
    detailed_timing:="$POINTPAINTING_DETAILED_TIMING" \
    "${EXTRA_LAUNCH_ARGS[@]}"
