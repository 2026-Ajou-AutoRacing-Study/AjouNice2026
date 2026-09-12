#!/usr/bin/env bash

set -euo pipefail

MODE="${1:-live}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

OPENPCDET_ROOT="${OPENPCDET_ROOT:-$HOME/OpenPCDet}"

MATP_MODEL_NAME="pointpillar_morai_pointpainting_base2_B0_3sweep_10feat_velocity_range30_matp_motion_rawgt_v2"

MATP_RUN_NAME="base2_B0_3sweep_matp_motion_rawgt_v2_80epoch_seed666_v1"

MATP_DEFAULT_CKPT="${OPENPCDET_ROOT}/output/custom_models/${MATP_MODEL_NAME}/${MATP_RUN_NAME}/ckpt/checkpoint_epoch_80.pth"

MATP_CKPT="${MATP_CKPT:-$MATP_DEFAULT_CKPT}"

export OPENPCDET_ROOT

export POINTPAINTING_CFG="${OPENPCDET_ROOT}/tools/cfgs/custom_models/${MATP_MODEL_NAME}.yaml"

export POINTPAINTING_CKPT="$MATP_CKPT"

# MATP = t-2, t-1, t
export POINTPAINTING_TEMPORAL_NUM_SWEEPS=3

# Reject broken / discontinuous temporal history.
export POINTPAINTING_TEMPORAL_MAXIMUM_GAP_SEC="${POINTPAINTING_TEMPORAL_MAXIMUM_GAP_SEC:-0.75}"

# MORAI LiDAR ~= 15 Hz.
export POINTPAINTING_TEMPORAL_LIDAR_SCAN_PERIOD_SEC="${POINTPAINTING_TEMPORAL_LIDAR_SCAN_PERIOD_SEC:-0.06666667}"

# Final MATP training convention:
# one measured time lag for each sweep.
export POINTPAINTING_TEMPORAL_TIME_LAG_MODE=sweep_constant

# MATP training used sweep-to-current ego-motion compensation.
# Per-scan rolling deskew is kept off for the first runtime validation.
export POINTPAINTING_LIDAR_DESKEW_MODE=off

# Publish MATP conv_vel -> pred_velocities -> DetectedObject twist.
export POINTPAINTING_PUBLISH_DETECTOR_VELOCITY=true

export POINTPAINTING_DETECTOR_VELOCITY_NOISE_STD_MPS="${POINTPAINTING_DETECTOR_VELOCITY_NOISE_STD_MPS:-1.0}"

export POINTPAINTING_DETECTOR_MAXIMUM_VELOCITY_MPS="${POINTPAINTING_DETECTOR_MAXIMUM_VELOCITY_MPS:-30.0}"

# Do not enable tracking/prediction wrapper here.
export POINTPAINTING_WITH_TP=false

export POINTPAINTING_BENCHMARK_LABEL="${POINTPAINTING_BENCHMARK_LABEL:-matp_rawgt_v2_e80}"

echo "=============================================="
echo " MATP ROS Runtime"
echo "=============================================="
echo "Mode       : $MODE"
echo "OpenPCDet  : $OPENPCDET_ROOT"
echo "Config     : $POINTPAINTING_CFG"
echo "Checkpoint : $POINTPAINTING_CKPT"
echo "Sweeps     : $POINTPAINTING_TEMPORAL_NUM_SWEEPS"
echo "Lag mode   : $POINTPAINTING_TEMPORAL_TIME_LAG_MODE"
echo "Velocity   : $POINTPAINTING_PUBLISH_DETECTOR_VELOCITY"
echo "Deskew     : $POINTPAINTING_LIDAR_DESKEW_MODE"
echo "=============================================="

if [[ ! -f "$POINTPAINTING_CFG" ]]; then
    echo "[ERROR] MATP config not found:"
    echo "  $POINTPAINTING_CFG"
    echo
    echo "Install the MATP OpenPCDet patch first."
    exit 1
fi

if [[ ! -f "$POINTPAINTING_CKPT" ]]; then
    echo "[ERROR] MATP checkpoint not found:"
    echo "  $POINTPAINTING_CKPT"
    echo
    echo "Override with:"
    echo '  export MATP_CKPT="/path/to/checkpoint_epoch_80.pth"'
    exit 1
fi

exec "$SCRIPT_DIR/run_pointpainting.sh" "$MODE"
