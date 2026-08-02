#!/usr/bin/env bash
# Upload Google Drive-managed K-City map assets and verify their checksums.

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

if [[ ! -d "${MAP_DIR}" ]]; then
  echo "ERROR: K-City map directory not found: ${MAP_DIR}" >&2
  exit 1
fi

if [[ ! -f "${FILTER_FILE}" ]]; then
  echo "ERROR: rclone filter not found: ${FILTER_FILE}" >&2
  exit 1
fi

mapfile -t ASSET_PATHS < <(
  rclone lsf "${MAP_DIR}" \
    --recursive \
    --files-only \
    --filter-from "${FILTER_FILE}" \
    | LC_ALL=C sort
)

if [[ "${#ASSET_PATHS[@]}" -eq 0 ]]; then
  echo "ERROR: No Google Drive-managed map assets were found in ${MAP_DIR}." >&2
  exit 1
fi

echo "Local map assets: ${#ASSET_PATHS[@]} files"
echo "Google Drive destination: ${REMOTE}"
echo "Google Drive root folder ID: ${DRIVE_ROOT_FOLDER_ID}"

rclone copy \
  "${MAP_DIR}" \
  "${REMOTE}" \
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
  echo "Dry run complete. No files were uploaded."
  exit 0
fi

echo "Upload complete. Verifying local assets against Google Drive..."
rclone check \
  "${MAP_DIR}" \
  "${REMOTE}" \
  --drive-root-folder-id "${DRIVE_ROOT_FOLDER_ID}" \
  --filter-from "${FILTER_FILE}" \
  --one-way \
  --checkers 4 \
  --tpslimit 5 \
  --tpslimit-burst 5

echo "K-City map upload and verification: OK"
