#!/usr/bin/env bash

# Run Base3-Static primary batches in one resumable session.  Only the current
# batch is installed in MORAI; completed batches are moved out of the live
# Scenario directory so the launcher preview does not accumulate thousands of
# entries.

set -Eeuo pipefail

PROJECT_ROOT="${HOME}/AjouNice2026"
START_BATCH=1
END_BATCH=7
SESSION_ROOT=""
RESUME=0
INSTALL_DIR="${HOME}/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
NETWORK_TEMPLATE="${INSTALL_DIR}/EgoNetwork/ros_path_probe_MN.json"

usage() {
  cat <<'EOF'
Usage:
  run_base3_static_10k_capture.sh [options]

Options:
  --session-root PATH  Shared output root for every collection phase.
  --start-batch N      First batch to capture, inclusive. Default: 1.
  --end-batch N        Last batch to capture, inclusive. Default: 7.
  --resume             Reopen an existing session root without overwriting batches.
  -h, --help           Show this help.
EOF
}

while (($#)); do
  case "$1" in
    --session-root)
      [[ $# -ge 2 ]] || { echo "--session-root requires a path" >&2; exit 2; }
      SESSION_ROOT="$2"
      shift 2
      ;;
    --start-batch)
      START_BATCH="$2"
      shift 2
      ;;
    --end-batch)
      END_BATCH="$2"
      shift 2
      ;;
    --resume)
      RESUME=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! [[ "${START_BATCH}" =~ ^[1-7]$ && "${END_BATCH}" =~ ^[1-7]$ ]]; then
  echo "Batch range must stay within 1..7" >&2
  exit 2
fi
if ((START_BATCH > END_BATCH)); then
  echo "--start-batch must be <= --end-batch" >&2
  exit 2
fi
if [[ -z "${SESSION_ROOT}" ]]; then
  if ((RESUME)); then
    echo "--resume requires an explicit --session-root" >&2
    exit 2
  fi
  SESSION_ROOT="${HOME}/morai_cap/$(date +%y%m%d_%H%M%S)_base3_static_10k"
fi

STATE_FILE="${SESSION_ROOT}/current_capture.tsv"
SESSION_MARKER="${SESSION_ROOT}/capture_session.tsv"
STATE_POINTER="/tmp/morai_base3_static_10k_session_dir"
ARCHIVE_ROOT="${HOME}/morai_scenario_archive/base3_static_10k/$(basename "${SESSION_ROOT}")"

if ((RESUME)); then
  [[ -f "${SESSION_MARKER}" ]] || {
    echo "Cannot resume: session marker is missing: ${SESSION_MARKER}" >&2
    exit 2
  }
  if ((START_BATCH > 1)); then
    for ((previous=1; previous<START_BATCH; previous++)); do
      previous_batch="$(printf '%03d' "${previous}")"
      previous_summary="${SESSION_ROOT}/batch${previous_batch}_primary/summary.json"
      [[ -f "${previous_summary}" ]] || {
        echo "Cannot resume: prior summary is missing: ${previous_summary}" >&2
        exit 2
      }
    done
  fi
else
  if [[ -e "${SESSION_ROOT}" ]] && [[ -n "$(find "${SESSION_ROOT}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    echo "Refusing to reuse non-empty session root without --resume: ${SESSION_ROOT}" >&2
    exit 2
  fi
  mkdir -p "${SESSION_ROOT}"
  printf 'schema_version\t1\ndataset_id\tmorai_static_classcentric_base3_10k\ncreated_at\t%s\n' \
    "$(date --iso-8601=seconds)" > "${SESSION_MARKER}"
fi

source /opt/ros/noetic/setup.bash
source "${PROJECT_ROOT}/base_ws/devel/setup.bash"
source "${HOME}/morai_personal_ws/devel/setup.bash"

printf '%s\n' "${SESSION_ROOT}" > "${STATE_POINTER}"
mkdir -p "${ARCHIVE_ROOT}"

write_state() {
  local batch="$1"
  local run_dir="$2"
  local manifest="$3"
  local temporary="${STATE_FILE}.tmp.$$"
  printf '%s\t%s\t%s\n' "${batch}" "${run_dir}" "${manifest}" > "${temporary}"
  mv -f "${temporary}" "${STATE_FILE}"
}

archive_manifest_scenes() {
  local manifest="$1"
  local batch="$2"
  local archive_dir="${ARCHIVE_ROOT}/batch${batch}"
  [[ -f "${manifest}" ]] || return 0
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

cd "${PROJECT_ROOT}"
echo "Base3-Static 10K Capture Session"
echo "Session root: ${SESSION_ROOT}"
echo "Batch range:  $(printf '%03d' "${START_BATCH}")..$(printf '%03d' "${END_BATCH}")"
echo "Resume:       ${RESUME}"

for ((batch_number=START_BATCH; batch_number<=END_BATCH; batch_number++)); do
  batch="$(printf '%03d' "${batch_number}")"
  source_manifest="${PROJECT_ROOT}/dataset/morai_od/manifests/static_classcentric_base3_batch${batch}_primary.json"
  runtime_manifest="${PROJECT_ROOT}/dataset/morai_od/manifests/static_classcentric_base3_batch${batch}_primary_runtime.json"
  plan="${PROJECT_ROOT}/dataset/morai_od/metadata/static_classcentric_base3_batches/batch${batch}/batch_plan.json"
  run_dir="${SESSION_ROOT}/batch${batch}_primary"

  [[ -f "${source_manifest}" ]] || { echo "Missing manifest: ${source_manifest}" >&2; exit 2; }
  [[ -f "${plan}" ]] || { echo "Missing plan: ${plan}" >&2; exit 2; }
  resume_args=()
  if [[ -e "${run_dir}" ]] && [[ -n "$(find "${run_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    if ((!RESUME)); then
      echo "Refusing to overwrite existing Batch ${batch}: ${run_dir}" >&2
      exit 2
    fi
    summary="${run_dir}/summary.json"
    [[ -f "${summary}" ]] || {
      echo "Cannot resume Batch ${batch}: summary is missing: ${summary}" >&2
      exit 2
    }
    summary_finished="$(python3 -c '
import json, sys
print("1" if json.load(open(sys.argv[1], encoding="utf-8")).get("finished") else "0")
' "${summary}")"
    if [[ "${summary_finished}" == "1" ]]; then
      echo "Batch ${batch} already finished; preserving it and continuing."
      archive_manifest_scenes "${runtime_manifest}" "${batch}"
      continue
    fi
    resume_args=(--resume-report "${summary}")
    echo "Resuming interrupted Batch ${batch} from ${summary}"
  fi

  python3 dataset/morai_od/scripts/install_dynamic_capture_set.py \
    "${source_manifest}" \
    --network-template "${NETWORK_TEMPLATE}" \
    --install-dir "${INSTALL_DIR}" \
    --runtime-manifest "${runtime_manifest}"

  mkdir -p "${run_dir}"
  write_state "${batch}" "${run_dir}" "${runtime_manifest}"

  echo
  echo "============================================================"
  echo "Starting Base3-Static Batch ${batch}/007"
  echo "Manifest: ${runtime_manifest}"
  echo "Output:   ${run_dir}"
  echo "============================================================"

  python3 -u dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
    --manifest "${runtime_manifest}" \
    --capture-dir "${run_dir}" \
    --no-timestamp-prefix \
    --motion-mode rolling-grouped \
    --scenario-load-attempts 3 \
    --capture-timeout-sec 30 \
    --continue-on-scene-error \
    "${resume_args[@]}" \
    2>&1 | tee -a "${run_dir}/console.log"

  archive_manifest_scenes "${runtime_manifest}" "${batch}"
done

if ((END_BATCH == 7)); then
  write_state "DONE" "${SESSION_ROOT}" "-"
  echo "All Base3-Static batches finished."
else
  next_batch="$(printf '%03d' "$((END_BATCH + 1))")"
  write_state "PAUSED" "${SESSION_ROOT}" "${next_batch}"
  echo "Requested range finished; next batch is ${next_batch}."
fi
echo "Session root: ${SESSION_ROOT}"
