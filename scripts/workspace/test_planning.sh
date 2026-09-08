#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export AJOUNICE_ROOT="$REPO_ROOT"
timeout_s="${AW149_PACKAGE_TEST_TIMEOUT_S:-600}"

if (( $# != 0 )); then
  echo "Usage: AW149_PACKAGE_TEST_TIMEOUT_S=600 $0" >&2
  exit 2
fi
if ! [[ "$timeout_s" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: AW149_PACKAGE_TEST_TIMEOUT_S must be a positive integer." >&2
  exit 2
fi

source /opt/ros/noetic/setup.bash
if [[ ! -f "$REPO_ROOT/planning_ws/install_isolated/setup.bash" ]]; then
  echo "ERROR: planning install is missing; run planning_ws/tools/build.sh --providers first." >&2
  exit 1
fi
source "$REPO_ROOT/planning_ws/install_isolated/setup.bash"

mapfile -t packages < <(
  catkin_topological_order --only-names "$REPO_ROOT/planning_ws/src" | LC_ALL=C sort
)
if (( ${#packages[@]} != 154 )); then
  echo "ERROR: expected 154 planning packages, found ${#packages[@]}." >&2
  printf '  %s\n' "${packages[@]}" >&2
  exit 1
fi

RUN_DIR="$(mktemp -d /tmp/aw149-planning-tests.XXXXXX)"
master_pid=""
cleanup_master() {
  if [[ -n "$master_pid" ]]; then
    kill -TERM "$master_pid" 2>/dev/null || true
    wait "$master_pid" 2>/dev/null || true
    master_pid=""
  fi
}
trap cleanup_master EXIT

for package in "${packages[@]}"; do
  build_dir="$REPO_ROOT/planning_ws/build_isolated/$package"
  if [[ ! -d "$build_dir" ]]; then
    echo "ERROR: missing build directory for $package; rebuild planning first." >&2
    exit 1
  fi

  package_run="$RUN_DIR/$package"
  mkdir -p "$package_run/ros_home" "$package_run/ros_log"
  cmake -E remove_directory "$build_dir/test_results"
  master_port="$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1]); s.close()')"
  export ROS_MASTER_URI="http://127.0.0.1:$master_port"
  export ROS_IP=127.0.0.1
  export ROS_HOME="$package_run/ros_home"
  export ROS_LOG_DIR="$package_run/ros_log"
  roscore -p "$master_port" >"$package_run/master.log" 2>&1 &
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
    echo "TEST_MASTER_FAIL $package log=$package_run/master.log" >&2
    exit 1
  fi

  printf 'TEST_START %s\n' "$package"
  if timeout "$timeout_s" make -C "$build_dir" run_tests -j1 >"$package_run/test.log" 2>&1; then
    printf 'TEST_TARGET_PASS %s\n' "$package"
  else
    result=$?
    printf 'TEST_TARGET_FAIL %s rc=%s log=%s\n' \
      "$package" "$result" "$package_run/test.log" >&2
    tail -100 "$package_run/test.log" >&2
    exit "$result"
  fi
  cleanup_master
done

python3 - "$REPO_ROOT/planning_ws/build_isolated" "${packages[@]}" <<'PY'
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

root = Path(sys.argv[1])
packages = sys.argv[2:]
totals = dict(packages=len(packages), packages_with_declared_tests=0,
              packages_without_declared_tests=0, junit_files=0, tests=0,
              errors=0, failures=0, skipped=0, malformed_junit_files=0)
for package in packages:
    files = sorted((root / package / "test_results" / package).rglob("*.xml"))
    if files:
        totals["packages_with_declared_tests"] += 1
    else:
        totals["packages_without_declared_tests"] += 1
    for path in files:
        totals["junit_files"] += 1
        try:
            node = ET.parse(path).getroot()
        except (ET.ParseError, OSError):
            totals["malformed_junit_files"] += 1
            continue
        for key in ("tests", "errors", "failures", "skipped"):
            totals[key] += int(node.attrib.get(key, 0))

print(json.dumps(totals, sort_keys=True))
if any(totals[key] for key in ("errors", "failures", "skipped", "malformed_junit_files")):
    raise SystemExit(1)
PY

printf 'AW149_PLANNING_TEST_TARGETS_PASS=%d/%d\n' "${#packages[@]}" "${#packages[@]}"
printf 'Logs: %s\n' "$RUN_DIR"
