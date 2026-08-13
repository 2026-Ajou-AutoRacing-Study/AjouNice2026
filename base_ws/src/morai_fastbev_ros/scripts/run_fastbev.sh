#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-live}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CATKIN_WS="$(cd "$PACKAGE_DIR/../.." && pwd)"
REPO_ROOT="$(cd "$CATKIN_WS/.." && pwd)"

CONDA_ENV="${CONDA_ENV:-fastbev_rtx50}"
FASTBEV_CONFIG="${FASTBEV_CONFIG:-$REPO_ROOT/perception/detection3d/fastbev/configs/fastbev_m0_morai_4cam_s384x704.py}"
FASTBEV_CHECKPOINT="${FASTBEV_CHECKPOINT:-$REPO_ROOT/artifacts/detection3d/fastbev/runs/full1500_stride1_fp16_wandb_v1/best_center_mAP_epoch_11.pth}"
FASTBEV_WITH_TP="${FASTBEV_WITH_TP:-false}"
FASTBEV_PREPROCESS_BACKEND="${FASTBEV_PREPROCESS_BACKEND:-opencv}"
FASTBEV_PIPELINE_MODE="${FASTBEV_PIPELINE_MODE:-staged}"

case "$MODE" in
  live) USE_SIM_TIME=false ;;
  bag) USE_SIM_TIME=true ;;
  *) echo "Usage: $0 [live|bag]" >&2; exit 2 ;;
esac

if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [[ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]]; then
  source "$HOME/anaconda3/etc/profile.d/conda.sh"
else
  echo "[ERROR] Conda initialization script not found." >&2
  exit 1
fi

set +u
conda activate "$CONDA_ENV"
set -u

source /opt/ros/noetic/setup.bash
if [[ ! -f "$CATKIN_WS/devel/setup.bash" ]]; then
  echo "[ERROR] Catkin workspace is not built: $CATKIN_WS" >&2
  exit 1
fi
source "$CATKIN_WS/devel/setup.bash"

for required in "$FASTBEV_CONFIG" "$FASTBEV_CHECKPOINT" "$REPO_ROOT/configs/sensors/Sensor_6.json"; do
  if [[ ! -f "$required" ]]; then
    echo "[ERROR] Required file not found: $required" >&2
    exit 1
  fi
done

# Preserve the ROS paths added by setup.bash while prepending the repository.
# Do not add /usr/lib/python3/dist-packages wholesale: Ubuntu 20.04's compiled
# Python 3.8 packages (notably NumPy) are incompatible with Conda Python 3.11.
export PYTHONPATH="$REPO_ROOT/third_party/fastbev:$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-fastbev-ros}"

# ROS Noetic binary extensions may resolve Conda's incompatible libffi first.
SYSTEM_LIBFFI="$(ldconfig -p | awk '/libffi\.so\.7/{print $NF; exit}')"
if [[ -n "$SYSTEM_LIBFFI" ]]; then
  export LD_PRELOAD="$SYSTEM_LIBFFI${LD_PRELOAD:+:$LD_PRELOAD}"
fi

if ! python -c 'import catkin_pkg, message_filters, netifaces, rospkg, rospy' >/dev/null 2>&1; then
  echo "[ERROR] Conda ROS Python dependencies are missing." >&2
  echo "        conda activate $CONDA_ENV" >&2
  echo "        python -m pip install rospkg catkin-pkg netifaces" >&2
  exit 1
fi

rosparam set /use_sim_time "$USE_SIM_TIME"

echo "=============================================="
echo "MORAI Fast-BEV ROS"
echo "=============================================="
echo "Mode              : $MODE"
echo "With T&P          : $FASTBEV_WITH_TP"
echo "Conda environment : $CONDA_ENV"
echo "Python            : $(command -v python)"
echo "Config            : $FASTBEV_CONFIG"
echo "Checkpoint        : $FASTBEV_CHECKPOINT"
echo "Preprocess        : $FASTBEV_PREPROCESS_BACKEND"
echo "Pipeline          : $FASTBEV_PIPELINE_MODE"
echo "use_sim_time      : $USE_SIM_TIME"
echo "=============================================="

if [[ "$FASTBEV_WITH_TP" == "true" ]]; then
  LAUNCH_FILE=fastbev_with_tp.launch
else
  LAUNCH_FILE=fastbev_online.launch
fi

exec roslaunch morai_fastbev_ros "$LAUNCH_FILE" \
  python_executable:="$(command -v python)" \
  repo_root:="$REPO_ROOT" \
  checkpoint:="$FASTBEV_CHECKPOINT" \
  preprocess_backend:="$FASTBEV_PREPROCESS_BACKEND" \
  pipeline_mode:="$FASTBEV_PIPELINE_MODE" \
  use_sim_time:="$USE_SIM_TIME"
