#!/usr/bin/env bash

# Capture the Base3-Online highway static Vehicle campaign in small MORAI
# installation shards.  The rosbag recorder is intentionally a separate
# terminal so one continuous split-bag run can span every shard.

set -Eeuo pipefail

ROOT="${AJOUNICE_ROOT:-${HOME}/AjouNice2026}"
INSTALL_DIR="${HOME}/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
NETWORK_TEMPLATE="${INSTALL_DIR}/EgoNetwork/ros_path_probe_MN.json"
SESSION_ROOT=""
START_BATCH=1
END_BATCH=4

usage() {
  cat <<'EOF'
Usage: run_base3_online_highway_static_v2_capture.sh [options]

Options:
  --session-root PATH  Runtime metadata output root.
  --start-batch N      First 250-scene shard, 1..4. Default: 1.
  --end-batch N        Last shard, 1..4. Default: 4.
  --install-dir PATH   MORAI scenario directory.
  --network-template PATH
                       Verified ROS EgoNetwork sidecar template.
  -h, --help           Show this help.

Start record_base3_online_paired_bag.sh in another terminal before this runner.
EOF
}

while (($#)); do
  case "$1" in
    --session-root) SESSION_ROOT="$2"; shift 2 ;;
    --start-batch) START_BATCH="$2"; shift 2 ;;
    --end-batch) END_BATCH="$2"; shift 2 ;;
    --install-dir) INSTALL_DIR="$2"; shift 2 ;;
    --network-template) NETWORK_TEMPLATE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ! [[ "${START_BATCH}" =~ ^[1-4]$ && "${END_BATCH}" =~ ^[1-4]$ ]]; then
  echo "Batch range must stay within 1..4" >&2
  exit 2
fi
if ((START_BATCH > END_BATCH)); then
  echo "--start-batch must be <= --end-batch" >&2
  exit 2
fi
if [[ -z "${SESSION_ROOT}" ]]; then
  SESSION_ROOT="${HOME}/morai_cap/$(date +%y%m%d_%H%M%S)_base3_online_highway_static_v2"
fi

source /opt/ros/noetic/setup.bash
if [[ -f "${ROOT}/base_ws/install_aw149/setup.bash" ]]; then
  source "${ROOT}/base_ws/install_aw149/setup.bash"
else
  source "${ROOT}/base_ws/devel/setup.bash"
fi
source "${HOME}/morai_personal_ws/devel/setup.bash"

rostopic list >/dev/null
[[ -f "${NETWORK_TEMPLATE}" ]] || {
  echo "Missing network template: ${NETWORK_TEMPLATE}" >&2
  exit 2
}

mkdir -p "${SESSION_ROOT}"
ARCHIVE_ROOT="${HOME}/morai_scenario_archive/base3_online_highway_static_v2/$(basename "${SESSION_ROOT}")"
mkdir -p "${ARCHIVE_ROOT}"

archive_manifest_scenes() {
  local manifest="$1"
  local batch="$2"
  local archive_dir="${ARCHIVE_ROOT}/batch${batch}"
  mkdir -p "${archive_dir}/EgoNetwork"
  while IFS= read -r stem; do
    [[ -n "${stem}" ]] || continue
    if [[ -f "${INSTALL_DIR}/${stem}.json" ]]; then
      mv -f "${INSTALL_DIR}/${stem}.json" "${archive_dir}/${stem}.json"
    fi
    if [[ -f "${INSTALL_DIR}/EgoNetwork/${stem}_MN.json" ]]; then
      mv -f "${INSTALL_DIR}/EgoNetwork/${stem}_MN.json" \
        "${archive_dir}/EgoNetwork/${stem}_MN.json"
    fi
  done < <(python3 -c '
import json, sys
for row in json.load(open(sys.argv[1], encoding="utf-8"))["scenes"]:
    print(row["file_name"])
' "${manifest}")
}

cd "${ROOT}"
printf 'dataset_id\tmorai_static_classcentric_base3_10k_online_highway_static_v2\n' \
  > "${SESSION_ROOT}/capture_session.tsv"
printf 'created_at\t%s\n' "$(date --iso-8601=seconds)" \
  >> "${SESSION_ROOT}/capture_session.tsv"

for ((number=START_BATCH; number<=END_BATCH; number++)); do
  batch="$(printf '%03d' "${number}")"
  source_manifest="${ROOT}/dataset/morai_od/manifests/base3_online_highway_static_v2_batch${batch}.json"
  runtime_manifest="${ROOT}/dataset/morai_od/manifests/base3_online_highway_static_v2_batch${batch}_runtime.json"
  run_dir="${SESSION_ROOT}/batch${batch}"

  [[ -f "${source_manifest}" ]] || {
    echo "Missing source manifest: ${source_manifest}" >&2
    exit 2
  }
  if [[ -e "${run_dir}/summary.json" ]]; then
    echo "Refusing to overwrite completed/partial shard output: ${run_dir}" >&2
    exit 2
  fi

  # Remove a stale installation of this same shard before copying it again.
  archive_manifest_scenes "${source_manifest}" "${batch}_stale"
  python3 dataset/morai_od/scripts/install_dynamic_capture_set.py \
    "${source_manifest}" \
    --network-template "${NETWORK_TEMPLATE}" \
    --online-bag-only \
    --install-dir "${INSTALL_DIR}" \
    --runtime-manifest "${runtime_manifest}"

  mkdir -p "${run_dir}"
  echo "Starting online highway static shard ${batch}/004"
  python3 -u dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
    --manifest "${runtime_manifest}" \
    --capture-dir "${run_dir}" \
    --no-timestamp-prefix \
    --motion-mode rolling-grouped \
    --online-bag-only \
    --scenario-load-attempts 3 \
    --capture-timeout-sec 30 \
    --continue-on-scene-error \
    2>&1 | tee -a "${run_dir}/console.log"

  archive_manifest_scenes "${runtime_manifest}" "${batch}"
done

echo "Capture shards finished: ${START_BATCH}..${END_BATCH}"
echo "Runtime metadata: ${SESSION_ROOT}"
echo "Stop the rosbag recorder with Ctrl-C now."
