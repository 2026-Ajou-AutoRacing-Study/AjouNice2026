#!/usr/bin/env bash
set -euo pipefail

# Run one ROS stack per planner-speed group, then combine the per-session
# summaries.  A separate stack is necessary because default_velocity_kph is a
# launch-time parameter, while each generated filename carries its ego speed.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SCENARIO_DIR="${MORAI_SCENARIO_DIR:-$HOME/Desktop/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025}"
SCENARIO_SERVICE="/Service_MoraiSL"
SCENARIO_TIMEOUT=120
EXECUTE=false
WITH_ROSBAG=false
SPEEDS=(5 10 20 30 40 50 60)

usage() {
  cat <<'EOF'
Usage: run_morai_scenario_batch_validation.sh [options]

Plans or runs all generated dynamic_*_pos001.json cases grouped by the
ego-speed encoded in each filename.  MORAI must be open, Ego Network must be
connected, and no other AJOUNICE ROS stack may be running when --execute is
used.

Options:
  --execute                 Run MORAI/ROS validation. Without this, print the plan.
  --with-rosbag             Record a bag for every case (off by default to save disk).
  --scenario-dir DIR        MORAI R_KR_PR_K-city_2025 scenario directory.
  --scenario-service NAME   MORAI service (default: /Service_MoraiSL).
  --scenario-timeout SEC    Per-case limit (default: 120).
  --speed KPH               Run only one speed group; may be repeated.
  -h, --help                Show this help.
EOF
}

selected_speeds=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      EXECUTE=true
      ;;
    --with-rosbag)
      WITH_ROSBAG=true
      ;;
    --scenario-dir)
      SCENARIO_DIR="$2"
      shift
      ;;
    --scenario-service)
      SCENARIO_SERVICE="$2"
      shift
      ;;
    --scenario-timeout)
      SCENARIO_TIMEOUT="$2"
      shift
      ;;
    --speed)
      selected_speeds+=("$2")
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ ${#selected_speeds[@]} -gt 0 ]]; then
  SPEEDS=("${selected_speeds[@]}")
fi
if [[ ! -d "$SCENARIO_DIR" ]]; then
  echo "MORAI scenario directory not found: $SCENARIO_DIR" >&2
  exit 1
fi

if [[ "$EXECUTE" != true ]]; then
  echo "DRY RUN: no MORAI/ROS process will be started."
  for speed in "${SPEEDS[@]}"; do
    printf -v speed_token '%02d' "$speed"
    echo "  ${speed} km/h: dynamic_*_ego${speed_token}*_pos001.json"
  done
  echo "Pass --execute after MORAI Ego Network is connected."
  exit 0
fi

batch_id="$(date +%Y%m%d_%H%M%S)"
host_batch_dir="$REPO_ROOT/artifacts/morai_scenario_validation/batches/$batch_id"
container_batch_dir="/home/ajou/AjouNice2026/artifacts/morai_scenario_validation/batches/$batch_id"
runner_batch_dir="$host_batch_dir"
if [[ "${AJOUNICE_ROS1_MODE:-auto}" == "docker" || ( "${AJOUNICE_ROS1_MODE:-auto}" == "auto" && ( ! -f /opt/ros/noetic/setup.bash || ! -f "$REPO_ROOT/base_ws/devel/setup.bash" ) ) ]]; then
  runner_batch_dir="$container_batch_dir"
fi
mkdir -p "$host_batch_dir"

overall_status=0
for speed in "${SPEEDS[@]}"; do
  printf -v speed_token '%02d' "$speed"
  echo "=== MORAI batch: ${speed} km/h ==="
  runner_args=(
    --execute
    --scenario-pattern "dynamic_*_ego${speed_token}*_pos001.json"
    --velocity-kph "$speed"
    --scenario-service "$SCENARIO_SERVICE"
    --scenario-timeout "$SCENARIO_TIMEOUT"
    --output-dir "$runner_batch_dir"
  )
  if [[ "$WITH_ROSBAG" != true ]]; then
    runner_args+=(--no-rosbag)
  fi
  if ! MORAI_SCENARIO_DIR="$SCENARIO_DIR" \
    "$SCRIPT_DIR/run_morai_scenario_validation.sh" "${runner_args[@]}"; then
    overall_status=1
  fi
done

if ! compgen -G "$host_batch_dir/*/summary.json" >/dev/null; then
  echo "No validation session was created; see the error above." >&2
  exit 1
fi

python3 "$SCRIPT_DIR/summarize_morai_scenario_validation.py" \
  --input-root "$host_batch_dir" \
  --output-dir "$host_batch_dir"
echo "Batch result table: $host_batch_dir/results.csv"
echo "Batch summary: $host_batch_dir/summary.md"
exit "$overall_status"
