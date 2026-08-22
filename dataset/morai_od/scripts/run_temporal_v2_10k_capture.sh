#!/usr/bin/env bash

# Run a contiguous range of Temporal V2 primary batches in one resumable
# session root and publish an atomic monitor state.

set -Eeuo pipefail

PROJECT_ROOT="${HOME}/AjouNice2026"
START_BATCH=1
END_BATCH=7
SESSION_ROOT=""
RESUME=0

usage() {
  cat <<'EOF'
Usage:
  run_temporal_v2_10k_capture.sh [options]

Options:
  --session-root PATH  Shared output root for every collection phase.
  --start-batch N      First batch to capture, inclusive. Default: 1.
  --end-batch N        Last batch to capture, inclusive. Default: 7.
  --resume             Reopen an existing session root without overwriting batches.
  -h, --help           Show this help.

Examples:
  # First phase.
  run_temporal_v2_10k_capture.sh --session-root /home/luv/morai_cap/run --start-batch 1 --end-batch 3

  # Second phase in the same session.
  run_temporal_v2_10k_capture.sh --session-root /home/luv/morai_cap/run --start-batch 4 --end-batch 7 --resume
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
      [[ $# -ge 2 ]] || { echo "--start-batch requires a number" >&2; exit 2; }
      START_BATCH="$2"
      shift 2
      ;;
    --end-batch)
      [[ $# -ge 2 ]] || { echo "--end-batch requires a number" >&2; exit 2; }
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

if [[ ! -d "${PROJECT_ROOT}/dataset/morai_od" ]]; then
  echo "AjouNice2026 dataset tree not found: ${PROJECT_ROOT}" >&2
  exit 2
fi
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
  SESSION_ROOT="${HOME}/morai_cap/$(date +%y%m%d_%H%M%S)_temporal_v2_base2_10k"
fi

STATE_FILE="${SESSION_ROOT}/current_capture.tsv"
SESSION_MARKER="${SESSION_ROOT}/capture_session.tsv"
STATE_POINTER="/tmp/morai_temporal_v2_10k_session_dir"

if ((RESUME)); then
  if [[ ! -f "${SESSION_MARKER}" ]]; then
    echo "Cannot resume: session marker is missing: ${SESSION_MARKER}" >&2
    exit 2
  fi
  if ((START_BATCH > 1)); then
    for ((previous=1; previous<START_BATCH; previous++)); do
      previous_batch="$(printf '%03d' "${previous}")"
      previous_summary="${SESSION_ROOT}/batch${previous_batch}_primary/summary.json"
      if [[ ! -f "${previous_summary}" ]]; then
        echo "Cannot resume at Batch $(printf '%03d' "${START_BATCH}"): prior summary is missing: ${previous_summary}" >&2
        exit 2
      fi
    done
  fi
else
  if [[ -e "${SESSION_ROOT}" ]] && [[ -n "$(find "${SESSION_ROOT}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    echo "Refusing to reuse non-empty session root without --resume: ${SESSION_ROOT}" >&2
    exit 2
  fi
  mkdir -p "${SESSION_ROOT}"
  printf 'schema_version\t1\nproject_root\t%s\ncreated_at\t%s\n' \
    "${PROJECT_ROOT}" "$(date --iso-8601=seconds)" > "${SESSION_MARKER}"
fi

source /opt/ros/noetic/setup.bash
source "${PROJECT_ROOT}/base_ws/devel/setup.bash"
source "${HOME}/morai_personal_ws/devel/setup.bash"

printf '%s\n' "${SESSION_ROOT}" > "${STATE_POINTER}"

write_state() {
  local batch="$1"
  local run_dir="$2"
  local manifest="$3"
  local temporary="${STATE_FILE}.tmp.$$"
  printf '%s\t%s\t%s\n' "${batch}" "${run_dir}" "${manifest}" > "${temporary}"
  mv -f "${temporary}" "${STATE_FILE}"
}

cd "${PROJECT_ROOT}"
echo "Temporal V2 Capture Session"
echo "Session root: ${SESSION_ROOT}"
echo "Batch range:  $(printf '%03d' "${START_BATCH}")..$(printf '%03d' "${END_BATCH}")"
echo "Resume:       ${RESUME}"

SESSION_SIGNATURE="$(awk -F '\t' '$1 == "baseline_signature" {print $2}' "${SESSION_MARKER}" | tail -n 1)"

for ((batch_number=START_BATCH; batch_number<=END_BATCH; batch_number++)); do
  batch="$(printf '%03d' "${batch_number}")"
  manifest="${PROJECT_ROOT}/dataset/morai_od/manifests/temporal_sequence_v2_batch${batch}_primary_runtime.json"
  plan="${PROJECT_ROOT}/dataset/morai_od/metadata/temporal_v2_batches/batch${batch}/batch_plan.json"
  run_dir="${SESSION_ROOT}/batch${batch}_primary"

  if [[ ! -f "${manifest}" ]]; then
    echo "Missing runtime manifest: ${manifest}" >&2
    exit 2
  fi
  if [[ ! -f "${plan}" ]]; then
    echo "Missing batch plan: ${plan}" >&2
    exit 2
  fi
  batch_signature="$(python3 -c '
import hashlib, json, sys
document = json.load(open(sys.argv[1], encoding="utf-8"))
primary = next(row for row in document["roles"] if row["role"] == "primary")
stable = {
    "campaign_tag": document["campaign_tag"],
    "seed_base": document["seed_base"],
    "canonical_frames_per_clip": document["canonical_frames_per_clip"],
    "family_clip_counts": primary["family_clip_counts"],
}
payload = json.dumps(stable, sort_keys=True, separators=(",", ":")).encode()
print(hashlib.sha256(payload).hexdigest())
' "${plan}")"
  if [[ -z "${SESSION_SIGNATURE}" ]]; then
    SESSION_SIGNATURE="${batch_signature}"
    printf 'baseline_signature\t%s\n' "${SESSION_SIGNATURE}" >> "${SESSION_MARKER}"
  elif [[ "${batch_signature}" != "${SESSION_SIGNATURE}" ]]; then
    echo "Batch ${batch} does not match the baseline used by this session." >&2
    echo "Expected signature: ${SESSION_SIGNATURE}" >&2
    echo "Actual signature:   ${batch_signature}" >&2
    exit 2
  fi
  if [[ -e "${run_dir}" ]] && [[ -n "$(find "${run_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    echo "Refusing to overwrite existing Batch ${batch}: ${run_dir}" >&2
    echo "Choose another range or remove the failed batch directory after review." >&2
    exit 2
  fi

  mkdir -p "${run_dir}"
  write_state "${batch}" "${run_dir}" "${manifest}"

  echo
  echo "============================================================"
  echo "Starting Batch ${batch}/007"
  echo "Manifest: ${manifest}"
  echo "Output:   ${run_dir}"
  echo "============================================================"

  python3 -u dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
    --manifest "${manifest}" \
    --capture-dir "${run_dir}" \
    --no-timestamp-prefix \
    --motion-mode timed-grouped \
    --scenario-load-attempts 2 \
    --continue-on-scene-error \
    2>&1 | tee "${run_dir}/console.log"
done

if ((END_BATCH == 7)); then
  write_state "DONE" "${SESSION_ROOT}" "-"
  echo
  echo "All requested batches finished; session is complete."
else
  next_batch="$(printf '%03d' "$((END_BATCH + 1))")"
  write_state "PAUSED" "${SESSION_ROOT}" "${next_batch}"
  echo
  echo "Requested range finished; session is paused."
  echo "Next batch: ${next_batch}"
fi
echo "Session root: ${SESSION_ROOT}"
