#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export AJOUNICE_ROOT="$REPO_ROOT"
source "$REPO_ROOT/scripts/workspace/source_aw149.sh"
set -u

master_port="${AW149_OUTPUT_PROBE_MASTER_PORT:-11555}"
timeout_s="${AW149_OUTPUT_PROBE_TIMEOUT_S:-240}"
if ! [[ "$master_port" =~ ^[1-9][0-9]*$ && "$timeout_s" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: master port and timeout must be positive integers." >&2
  exit 2
fi

RUN_DIR="$(mktemp -d /tmp/aw149-integrated-output.XXXXXX)"
export ROS_HOME="$RUN_DIR/ros_home"
export ROS_LOG_DIR="$RUN_DIR/ros_log"
export ROS_MASTER_URI="http://127.0.0.1:$master_port"
export ROS_IP=127.0.0.1
export AW149_OUTPUT_PROBE_RESULT="$RUN_DIR/result.json"

cleanup() {
  set +e
  if [[ -n "${master_pid:-}" ]]; then
    kill -TERM "$master_pid" 2>/dev/null
    wait "$master_pid" 2>/dev/null
  fi
}
trap cleanup EXIT

roscore -p "$master_port" >"$RUN_DIR/master.log" 2>&1 &
master_pid=$!
ready=false
for _ in $(seq 1 50); do
  if rosparam list >/dev/null 2>&1; then
    ready=true
    break
  fi
  sleep 0.1
done
if [[ "$ready" != "true" ]]; then
  echo "ERROR: isolated ROS master did not become ready. Logs: $RUN_DIR" >&2
  exit 1
fi

if timeout "$timeout_s" python3 \
  "$REPO_ROOT/scripts/runtime/verify_integrated_planning_output.py" \
  >"$RUN_DIR/probe.log" 2>&1; then
  :
else
  code=$?
  echo "ERROR: integrated output probe failed with code $code. Logs: $RUN_DIR" >&2
  tail -120 "$RUN_DIR/probe.log" >&2
  exit "$code"
fi

cat "$AW149_OUTPUT_PROBE_RESULT"
echo "AW149_INTEGRATED_OUTPUT_PASS"
echo "logs: $RUN_DIR"
