#!/usr/bin/env bash

set -euo pipefail

# ============================================================
# MATP OpenPCDet patch installer
#
# Usage:
#   OPENPCDET_ROOT=/path/to/OpenPCDet \
#   ./install_matp_openpcdet.sh
#
# Default:
#   OPENPCDET_ROOT=$HOME/OpenPCDet
# ============================================================

OPENPCDET_ROOT="${OPENPCDET_ROOT:-$HOME/OpenPCDet}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH_ROOT="$(cd "${SCRIPT_DIR}/../openpcdet_patch" && pwd)"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP_ROOT="${OPENPCDET_ROOT}/.matp_backup/${TIMESTAMP}"

echo "================================================"
echo " MATP OpenPCDet patch installer"
echo "================================================"
echo "OpenPCDet : ${OPENPCDET_ROOT}"
echo "Patch     : ${PATCH_ROOT}"
echo "Backup    : ${BACKUP_ROOT}"
echo

# ------------------------------------------------------------
# Validate OpenPCDet root
# ------------------------------------------------------------

if [[ ! -d "${OPENPCDET_ROOT}/pcdet" ]]; then
    echo "[ERROR] OpenPCDet pcdet directory not found:"
    echo "  ${OPENPCDET_ROOT}/pcdet"
    exit 1
fi

if [[ ! -d "${OPENPCDET_ROOT}/tools" ]]; then
    echo "[ERROR] OpenPCDet tools directory not found:"
    echo "  ${OPENPCDET_ROOT}/tools"
    exit 1
fi

if [[ ! -d "${PATCH_ROOT}" ]]; then
    echo "[ERROR] MATP patch directory not found:"
    echo "  ${PATCH_ROOT}"
    exit 1
fi

# ------------------------------------------------------------
# Files required by final MATP rawgt_v2 model
# ------------------------------------------------------------

FILES=(
    "pcdet/datasets/__init__.py"
    "pcdet/datasets/processor/data_processor.py"

    "pcdet/models/backbones_2d/map_to_bev/__init__.py"
    "pcdet/models/backbones_2d/map_to_bev/temporal_pointpillar_scatter.py"
    "pcdet/models/backbones_2d/map_to_bev/matp_motion_pointpillar_scatter.py"

    "pcdet/models/dense_heads/anchor_head_single.py"
    "pcdet/models/dense_heads/target_assigner/axis_aligned_target_assigner.py"

    "pcdet/models/detectors/detector3d_template.py"

    "tools/cfgs/custom_models/pointpillar_morai_pointpainting_base2_B0_3sweep_10feat_velocity_range30_matp_motion_rawgt_v2.yaml"

    "tools/cfgs/dataset_configs/morai_pointpainting_base2_B0_3sweep_10feat_velocity_range30_tfeature_rawgt_v2.yaml"
)

# ------------------------------------------------------------
# Verify every source patch file exists before touching OpenPCDet
# ------------------------------------------------------------

echo "[CHECK] Patch files"

for REL_PATH in "${FILES[@]}"; do
    SRC="${PATCH_ROOT}/${REL_PATH}"

    if [[ ! -f "${SRC}" ]]; then
        echo "[ERROR] Missing patch file:"
        echo "  ${SRC}"
        exit 1
    fi

    echo "  [OK] ${REL_PATH}"
done

echo

# ------------------------------------------------------------
# Install files
# ------------------------------------------------------------

mkdir -p "${BACKUP_ROOT}"

for REL_PATH in "${FILES[@]}"; do
    SRC="${PATCH_ROOT}/${REL_PATH}"
    DST="${OPENPCDET_ROOT}/${REL_PATH}"

    DST_DIR="$(dirname "${DST}")"
    BACKUP_FILE="${BACKUP_ROOT}/${REL_PATH}"
    BACKUP_DIR="$(dirname "${BACKUP_FILE}")"

    mkdir -p "${DST_DIR}"

    if [[ -f "${DST}" ]]; then
        mkdir -p "${BACKUP_DIR}"
        cp -a "${DST}" "${BACKUP_FILE}"

        echo "[BACKUP] ${REL_PATH}"
    fi

    cp -a "${SRC}" "${DST}"

    echo "[INSTALL] ${REL_PATH}"
done

echo

# ------------------------------------------------------------
# Python syntax validation
# ------------------------------------------------------------

echo "[CHECK] Python syntax"

PYTHON_BIN="${PYTHON_BIN:-python3}"

"${PYTHON_BIN}" -m py_compile \
    "${OPENPCDET_ROOT}/pcdet/datasets/__init__.py" \
    "${OPENPCDET_ROOT}/pcdet/datasets/processor/data_processor.py" \
    "${OPENPCDET_ROOT}/pcdet/models/backbones_2d/map_to_bev/__init__.py" \
    "${OPENPCDET_ROOT}/pcdet/models/backbones_2d/map_to_bev/temporal_pointpillar_scatter.py" \
    "${OPENPCDET_ROOT}/pcdet/models/backbones_2d/map_to_bev/matp_motion_pointpillar_scatter.py" \
    "${OPENPCDET_ROOT}/pcdet/models/dense_heads/anchor_head_single.py" \
    "${OPENPCDET_ROOT}/pcdet/models/dense_heads/target_assigner/axis_aligned_target_assigner.py" \
    "${OPENPCDET_ROOT}/pcdet/models/detectors/detector3d_template.py"

echo

# ------------------------------------------------------------
# Final summary
# ------------------------------------------------------------

echo "================================================"
echo " MATP patch installation complete"
echo "================================================"
echo
echo "Backup:"
echo "  ${BACKUP_ROOT}"
echo
echo "Model config:"
echo "  tools/cfgs/custom_models/"
echo "  pointpillar_morai_pointpainting_base2_B0_3sweep_10feat_velocity_range30_matp_motion_rawgt_v2.yaml"
echo
echo "Dataset config:"
echo "  tools/cfgs/dataset_configs/"
echo "  morai_pointpainting_base2_B0_3sweep_10feat_velocity_range30_tfeature_rawgt_v2.yaml"
echo
echo "MATP:"
echo "  sweeps          : 3"
echo "  point features  : 10"
echo "  MAP_TO_BEV      : MATPMotionPointPillarScatter"
echo "  velocity output : vx, vy"
echo
echo "NOTE:"
echo "  - checkpoint_epoch_80.pth is not copied by this installer."
echo "  - OpenPCDet CUDA extensions must already be built."
echo "  - pcdet/version.py is generated by the OpenPCDet installation."
echo "================================================"
