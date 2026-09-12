#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-live}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CATKIN_WS="$(cd "$PACKAGE_DIR/../.." && pwd)"

CONDA_ENV="${CONDA_ENV:-openpcdet_team}"
POINTPAINTING_ACTIVATE_CONDA="${POINTPAINTING_ACTIVATE_CONDA:-true}"
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
POINTPAINTING_SCORE_THRESHOLD="${POINTPAINTING_SCORE_THRESHOLD:-0.10}"
POINTPAINTING_VEHICLE_SCORE_THRESHOLD="${POINTPAINTING_VEHICLE_SCORE_THRESHOLD:-0.40}"
POINTPAINTING_PEDESTRIAN_SCORE_THRESHOLD="${POINTPAINTING_PEDESTRIAN_SCORE_THRESHOLD:-0.15}"
POINTPAINTING_OBSTACLE_SCORE_THRESHOLD="${POINTPAINTING_OBSTACLE_SCORE_THRESHOLD:-0.10}"
POINTPAINTING_LIDAR_MEASUREMENT_STAMP_MODE="${POINTPAINTING_LIDAR_MEASUREMENT_STAMP_MODE:-object_points_median}"
POINTPAINTING_LIDAR_DESKEW_MODE="${POINTPAINTING_LIDAR_DESKEW_MODE:-off}"
POINTPAINTING_LIDAR_DESKEW_REFERENCE="${POINTPAINTING_LIDAR_DESKEW_REFERENCE:-point_median}"
POINTPAINTING_LIDAR_DESKEW_CAMERA_REFERENCE="${POINTPAINTING_LIDAR_DESKEW_CAMERA_REFERENCE:-reference}"
POINTPAINTING_TEMPORAL_NUM_SWEEPS="${POINTPAINTING_TEMPORAL_NUM_SWEEPS:-0}"
POINTPAINTING_TEMPORAL_MAXIMUM_GAP_SEC="${POINTPAINTING_TEMPORAL_MAXIMUM_GAP_SEC:-0.75}"
POINTPAINTING_TEMPORAL_LIDAR_SCAN_PERIOD_SEC="${POINTPAINTING_TEMPORAL_LIDAR_SCAN_PERIOD_SEC:-0.06666667}"
POINTPAINTING_TEMPORAL_TIME_LAG_MODE="${POINTPAINTING_TEMPORAL_TIME_LAG_MODE:-sweep_constant}"
POINTPAINTING_ORACLE_PAINTING_CLASS="${POINTPAINTING_ORACLE_PAINTING_CLASS:-}"
POINTPAINTING_ORACLE_PAINTING_MODE="${POINTPAINTING_ORACLE_PAINTING_MODE:-one_hot}"

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

if [[ "$POINTPAINTING_ACTIVATE_CONDA" == "true" ]]; then
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
elif [[ -z "${CONDA_PREFIX:-}" || -z "${CONDA_DEFAULT_ENV:-}" ]]; then
    echo "[ERROR] PointPainting Conda 환경을 먼저 활성화하세요:"
    echo "  conda activate pp"
    exit 1
else
    CONDA_ENV="$CONDA_DEFAULT_ENV"
fi

AW149_SETUP="$REPO_ROOT/scripts/workspace/source_aw149.sh"
if [[ ! -f "$AW149_SETUP" ]]; then
    echo "[ERROR] AW149 runtime setup을 찾을 수 없습니다:"
    echo "  $AW149_SETUP"
    exit 1
fi

export AJOUNICE_ROOT="$REPO_ROOT"
source "$AW149_SETUP"

# ROS Noetic's Debian Python modules (for example rospkg) live outside the
# Conda environment. Add the Conda site-packages explicitly before Debian's
# directory: PYTHONPATH entries otherwise precede normal site initialization,
# causing the system NumPy to shadow the model environment's ABI-compatible
# NumPy/OpenCV/SharedArray stack.
CONDA_SITE_PACKAGES="$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$CONDA_SITE_PACKAGES:/usr/lib/python3/dist-packages"
unset CONDA_SITE_PACKAGES

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
echo "Conda auto-activate: $POINTPAINTING_ACTIVATE_CONDA"
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
echo "LiDAR stamp mode  : $POINTPAINTING_LIDAR_MEASUREMENT_STAMP_MODE"
echo "LiDAR deskew      : $POINTPAINTING_LIDAR_DESKEW_MODE ($POINTPAINTING_LIDAR_DESKEW_REFERENCE, camera=$POINTPAINTING_LIDAR_DESKEW_CAMERA_REFERENCE)"
echo "Temporal sweeps   : $POINTPAINTING_TEMPORAL_NUM_SWEEPS (max_gap=$POINTPAINTING_TEMPORAL_MAXIMUM_GAP_SEC, lag=$POINTPAINTING_TEMPORAL_TIME_LAG_MODE)"
echo "Oracle painting   : ${POINTPAINTING_ORACLE_PAINTING_CLASS:-disabled} ($POINTPAINTING_ORACLE_PAINTING_MODE, diagnostic only)"
echo "use_sim_time      : $USE_SIM_TIME"
echo "=============================================="

if [[ "$POINTPAINTING_WITH_TP" == "true" ]]; then
    LAUNCH_FILE=pointpainting_with_tp.launch
    EXTRA_LAUNCH_ARGS=(
        prediction_model:="$POINTPAINTING_PREDICTION_MODEL"
    )
else
    LAUNCH_FILE=pointpainting_online.launch
    EXTRA_LAUNCH_ARGS=(
        oracle_painting_class:="$POINTPAINTING_ORACLE_PAINTING_CLASS"
        oracle_painting_mode:="$POINTPAINTING_ORACLE_PAINTING_MODE"
    )
fi

# Run roslaunch with the same Conda interpreter.  Invoking the Noetic script
# through its Python 3.8 shebang would make it import Python 3.11 packages from
# the PointPainting PYTHONPATH and fail before the detector starts.
exec "$(command -v python)" "$(command -v roslaunch)" morai_pointpainting_ros \
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
    score_threshold:="$POINTPAINTING_SCORE_THRESHOLD" \
    vehicle_score_threshold:="$POINTPAINTING_VEHICLE_SCORE_THRESHOLD" \
    pedestrian_score_threshold:="$POINTPAINTING_PEDESTRIAN_SCORE_THRESHOLD" \
    obstacle_score_threshold:="$POINTPAINTING_OBSTACLE_SCORE_THRESHOLD" \
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
    lidar_measurement_stamp_mode:="$POINTPAINTING_LIDAR_MEASUREMENT_STAMP_MODE" \
    lidar_deskew_mode:="$POINTPAINTING_LIDAR_DESKEW_MODE" \
    lidar_deskew_reference:="$POINTPAINTING_LIDAR_DESKEW_REFERENCE" \
    lidar_deskew_camera_reference:="$POINTPAINTING_LIDAR_DESKEW_CAMERA_REFERENCE" \
    temporal_num_sweeps:="$POINTPAINTING_TEMPORAL_NUM_SWEEPS" \
    temporal_maximum_gap_sec:="$POINTPAINTING_TEMPORAL_MAXIMUM_GAP_SEC" \
    temporal_lidar_scan_period_sec:="$POINTPAINTING_TEMPORAL_LIDAR_SCAN_PERIOD_SEC" \
    temporal_time_lag_mode:="$POINTPAINTING_TEMPORAL_TIME_LAG_MODE" \
    "${EXTRA_LAUNCH_ARGS[@]}"
