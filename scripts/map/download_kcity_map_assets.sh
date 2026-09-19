#!/usr/bin/env bash
# Download Google Drive-managed K-City map assets and verify remote checksums.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MAP_DIR="${REPO_ROOT}/kcity_map"
FILTER_FILE="${REPO_ROOT}/configs/map/kcity_map_drive.filter"

RCLONE_REMOTE="${KCITY_MAP_RCLONE_REMOTE:-ajou_drive}"
DRIVE_ROOT_FOLDER_ID="${KCITY_MAP_DRIVE_ROOT_FOLDER_ID:-12T1IuT2ZyIYG5IkMDvsPrN1zzf6O43IP}"
REMOTE_MAP_PATH="${KCITY_MAP_DRIVE_PATH:-kcity_map}"
REMOTE="${RCLONE_REMOTE}:${REMOTE_MAP_PATH}"

DRY_RUN=false
for arg in "$@"; do
  if [[ "${arg}" == "--dry-run" || "${arg}" == "-n" ]]; then
    DRY_RUN=true
  fi
done

if ! command -v rclone >/dev/null 2>&1; then
  echo "ERROR: rclone is not installed." >&2
  echo "Install it with: sudo apt update && sudo apt install -y rclone" >&2
  exit 1
fi

if [[ ! -f "${FILTER_FILE}" ]]; then
  echo "ERROR: rclone filter not found: ${FILTER_FILE}" >&2
  exit 1
fi

mkdir -p "${MAP_DIR}"

echo "Google Drive source: ${REMOTE}"
echo "Google Drive root folder ID: ${DRIVE_ROOT_FOLDER_ID}"
echo "Local destination: ${MAP_DIR}"

rclone copy \
  "${REMOTE}" \
  "${MAP_DIR}" \
  --drive-root-folder-id "${DRIVE_ROOT_FOLDER_ID}" \
  --filter-from "${FILTER_FILE}" \
  --checksum \
  --immutable \
  --transfers 4 \
  --checkers 4 \
  --tpslimit 5 \
  --tpslimit-burst 5 \
  --progress \
  "$@"

if [[ "${DRY_RUN}" == "true" ]]; then
  echo "Dry run complete. No files were downloaded."
  exit 0
fi

echo "Download complete. Verifying Google Drive assets against local files..."
rclone check \
  "${REMOTE}" \
  "${MAP_DIR}" \
  --drive-root-folder-id "${DRIVE_ROOT_FOLDER_ID}" \
  --filter-from "${FILTER_FILE}" \
  --one-way \
  --checkers 4 \
  --tpslimit 5 \
  --tpslimit-burst 5

echo "K-City map download and verification: OK"
