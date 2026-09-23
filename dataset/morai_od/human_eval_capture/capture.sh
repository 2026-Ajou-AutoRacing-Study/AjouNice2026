#!/usr/bin/env bash

# Small session manager for a human-driven raw rosbag.  It intentionally does
# not start MORAI, a detector, tracking, planning, or control.

set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/common.sh"

pid_file="$runtime_dir/active.pid"
prefix_file="$runtime_dir/active.prefix"
metadata_file="$runtime_dir/active.metadata"
log_file_pointer="$runtime_dir/active.log"

usage() {
  cat <<EOF
usage:
  $0 list
  $0 start RUN_ID [--driver NAME] [--output-root PATH]
  $0 status
  $0 stop
  $0 verify BAG [BAG ...]

Examples:
  $0 list
  $0 start H01 --driver youngjae
  $0 stop
EOF
}

lookup_run() {
  python3 - "$plan_file" "$1" <<'PY'
import json, sys
plan = json.load(open(sys.argv[1], encoding="utf-8"))
run_id = sys.argv[2].upper()
for item in plan["runs"]:
    if item["id"].upper() == run_id:
        print("\t".join(str(item[key]) for key in
              ("id", "mission", "role", "scenario", "instruction")))
        break
else:
    raise SystemExit("unknown run ID: {}".format(run_id))
PY
}

list_runs() {
  python3 - "$plan_file" <<'PY'
import json, sys
plan = json.load(open(sys.argv[1], encoding="utf-8"))
print("ID   P  mission      role                scenario")
for item in sorted(plan["runs"], key=lambda x: (x["priority"], x["id"])):
    print("{:<4} {:<2} {:<12} {:<19} {}".format(
        item["id"], item["priority"], item["mission"], item["role"],
        item["scenario"]))
PY
}

active_pid() {
  [[ -f "$pid_file" ]] || return 1
  local pid
  pid="$(cat "$pid_file")"
  kill -0 "$pid" 2>/dev/null || return 1
  printf '%s\n' "$pid"
}

start_capture() {
  local run_id="${1:-}"
  [[ -n "$run_id" ]] || { usage; exit 2; }
  shift || true
  local driver="unknown"
  local output_root="${HUMAN_EVAL_BAG_ROOT:-$HOME/ajounice_human_eval_bags}"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --driver) driver="${2:?missing driver}"; shift 2 ;;
      --output-root) output_root="${2:?missing output root}"; shift 2 ;;
      *) echo "ERROR: unknown option: $1" >&2; usage; exit 2 ;;
    esac
  done
  if pid="$(active_pid)"; then
    echo "ERROR: capture already active (PID $pid). Run '$0 status' or '$0 stop'." >&2
    exit 2
  fi
  rm -f "$pid_file" "$prefix_file" "$metadata_file" "$log_file_pointer"

  local row
  row="$(lookup_run "$run_id")"
  local canonical_id mission role scenario instruction
  IFS=$'\t' read -r canonical_id mission role scenario instruction <<<"$row"
  local scenario_path="$scenario_source_dir/$scenario"
  [[ -f "$scenario_path" ]] || {
    echo "ERROR: scenario missing from Git checkout: $scenario_path" >&2
    exit 2
  }

  source_capture_workspace
  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
  python3 "$AJOUNICE_ROOT/dataset/morai_od/scripts/check_base3_online_runtime.py" \
    --gt-transport udp --message-timeout-sec 5.0

  local free_kib
  mkdir -p "$output_root/$mission"
  free_kib="$(df -Pk "$output_root" | awk 'NR==2 {print $4}')"
  if (( free_kib < 30 * 1024 * 1024 )); then
    echo "ERROR: less than 30 GiB free under $output_root" >&2
    exit 2
  fi

  local stamp run_dir prefix log metadata
  stamp="$(date +%y%m%d_%H%M%S)"
  run_dir="$output_root/$mission/${stamp}_${canonical_id}_${role}"
  mkdir -p "$run_dir"
  prefix="$run_dir/${stamp}_${canonical_id}_${scenario%.json}"
  log="${prefix}_rosbag.log"
  metadata="${prefix}_metadata.json"

  python3 - "$plan_file" "$canonical_id" "$driver" "$scenario_path" \
    "$metadata" "$AJOUNICE_ROOT" <<'PY'
import datetime, hashlib, json, subprocess, sys
plan_path, run_id, driver, scenario_path, output, root = sys.argv[1:]
plan = json.load(open(plan_path, encoding="utf-8"))
run = next(item for item in plan["runs"] if item["id"] == run_id)
with open(scenario_path, "rb") as stream:
    scenario_sha = hashlib.sha256(stream.read()).hexdigest()
git_sha = subprocess.check_output(
    ["git", "-C", root, "rev-parse", "HEAD"], text=True).strip()
document = {
    "schema_version": 1,
    "status": "recording",
    "started_at": datetime.datetime.now(datetime.timezone.utc).astimezone().isoformat(),
    "driver": driver,
    "git_sha": git_sha,
    "run": run,
    "scenario_sha256": scenario_sha,
    "sensor_config": "configs/sensors/Sensor_competition_local.json",
    "network_config": "configs/network/ego_network_competition_local.json",
    "gt_transport": "udp",
    "detector_running": False,
}
with open(output, "w", encoding="utf-8") as stream:
    json.dump(document, stream, indent=2, ensure_ascii=False)
    stream.write("\n")
PY

  local -a topics=(
    /sensor/lidar1
    /sensor/camera/front/compressed
    /sensor/camera/front_left/compressed
    /sensor/camera/front_right/compressed
    /sensor/camera/back/compressed
    /Ego_topic
    /Object_topic
    /Competition_topic
    /sensor/gps
    /sensor/imu
    /CollisionData
    /VehicleCollisionData
    /IntscnTL_topic
    /GetTrafficLightStatus
    /ReplayInfo_topic
    /lamps
    /InsnControl
    /InsnStatus
    /SyncModeInfo
    /sim/process/state/msg/MoraiSimProcStatus
    /ctrl_cmd
    /tf
    /tf_static
    /clock
  )

  setsid rosbag record --lz4 --buffsize=4096 --split --size=4096 \
    -O "$prefix" "${topics[@]}" >"$log" 2>&1 < /dev/null &
  local pid=$!
  printf '%s\n' "$pid" > "$pid_file"
  printf '%s\n' "$prefix" > "$prefix_file"
  printf '%s\n' "$metadata" > "$metadata_file"
  printf '%s\n' "$log" > "$log_file_pointer"
  sleep 2
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "ERROR: rosbag recorder exited during startup." >&2
    tail -n 50 "$log" >&2 || true
    rm -f "$pid_file" "$prefix_file" "$metadata_file" "$log_file_pointer"
    exit 2
  fi

  cat <<EOF
CAPTURE STARTED
  run      : $canonical_id ($mission / $role)
  scenario : $scenario
  output   : $run_dir
  recorder : PID $pid

MR에서 위 scenario가 실제로 load됐는지 확인한 뒤 주행하세요.
주행 지시: $instruction

종료 명령:
  $0 stop
EOF
}

status_capture() {
  if pid="$(active_pid)"; then
    echo "ACTIVE: PID $pid"
    echo "prefix: $(cat "$prefix_file")"
    echo "metadata: $(cat "$metadata_file")"
    echo "recent recorder log:"
    tail -n 12 "$(cat "$log_file_pointer")" || true
  else
    echo "No active human-evaluation capture."
    [[ ! -f "$pid_file" ]] || echo "A stale state file exists and will be cleared by the next start."
  fi
}

stop_capture() {
  local pid
  if ! pid="$(active_pid)"; then
    echo "ERROR: no active capture." >&2
    exit 2
  fi
  local prefix metadata
  prefix="$(cat "$prefix_file")"
  metadata="$(cat "$metadata_file")"
  echo "Stopping rosbag process group $pid with SIGINT..."
  kill -INT -- "-$pid"
  local remaining=90
  while kill -0 "$pid" 2>/dev/null && (( remaining > 0 )); do
    sleep 1
    remaining=$((remaining - 1))
  done
  if kill -0 "$pid" 2>/dev/null; then
    echo "ERROR: recorder did not close within 90 seconds." >&2
    echo "Do not force-kill it until disk activity stops; inspect: $(cat "$log_file_pointer")" >&2
    exit 2
  fi

  mapfile -t bags < <(find "$(dirname "$prefix")" -maxdepth 1 -type f \
    -name "$(basename "$prefix")*.bag" | sort)
  if [[ ${#bags[@]} -eq 0 ]]; then
    echo "ERROR: recorder closed but no finalized .bag was found." >&2
    exit 2
  fi
  local report="${prefix}_audit.json"
  local audit_status=0
  python3 "$HERE/verify_bags.py" "${bags[@]}" --json-output "$report" || audit_status=$?

  python3 - "$metadata" "$report" "${bags[@]}" <<'PY'
import datetime, json, sys
metadata, report, *bags = sys.argv[1:]
document = json.load(open(metadata, encoding="utf-8"))
audit = json.load(open(report, encoding="utf-8"))
document.update({
    "status": "complete" if audit["passed"] else "audit_failed",
    "ended_at": datetime.datetime.now(datetime.timezone.utc).astimezone().isoformat(),
    "bags": bags,
    "audit_report": report,
    "audit_passed": bool(audit["passed"]),
})
with open(metadata, "w", encoding="utf-8") as stream:
    json.dump(document, stream, indent=2, ensure_ascii=False)
    stream.write("\n")
PY
  rm -f "$pid_file" "$prefix_file" "$metadata_file" "$log_file_pointer"
  echo "Capture finalized: $(dirname "$prefix")"
  return "$audit_status"
}

verify_capture() {
  [[ $# -gt 0 ]] || { usage; exit 2; }
  source_capture_workspace
  python3 "$HERE/verify_bags.py" "$@"
}

command="${1:-}"
shift || true
case "$command" in
  list) list_runs ;;
  start) start_capture "$@" ;;
  status) status_capture ;;
  stop) stop_capture ;;
  verify) verify_capture "$@" ;;
  *) usage; exit 2 ;;
esac
