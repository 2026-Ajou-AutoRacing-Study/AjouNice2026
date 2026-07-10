#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_mgeo_controller_tuning.sh <profile_name> [profile_yaml]

Examples:
  ~/catkin_ws/src/ajou_control/scripts/run_mgeo_controller_tuning.sh baseline
  ~/catkin_ws/src/ajou_control/scripts/run_mgeo_controller_tuning.sh max_steer_rate_08

Each profile is loaded from:
  ~/catkin_ws/src/ajou_control/configs/controller_tuning/<profile_name>.yaml

The validation result is saved to:
  ~/catkin_ws/src/ajou_control/validation_runs/<profile_name>.csv

Set COMPARE_WITH to overlay selected previous runs while driving:
  COMPARE_WITH=$HOME/catkin_ws/src/ajou_control/validation_runs/baseline.csv \
    run_mgeo_controller_tuning.sh max_steer_rate_08
EOF
}

yaml_value() {
  local key="$1"
  local file="$2"
  awk -F: -v key="$key" '
    $1 ~ "^[[:space:]]*" key "[[:space:]]*$" {
      value=$0
      sub(/^[^:]*:/, "", value)
      sub(/[[:space:]]+#.*$/, "", value)
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
      gsub(/^["'\'']|["'\'']$/, "", value)
      print value
      exit
    }
  ' "$file"
}

config_value() {
  local env_name="$1"
  local yaml_key="$2"
  local default_value="$3"
  local value="${!env_name:-}"
  if [ -z "$value" ]; then
    value="$(yaml_value "$yaml_key" "$CONFIG_FILE")"
  fi
  if [ -z "$value" ]; then
    value="$default_value"
  fi
  printf '%s' "$value"
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

RUN_NAME="${1:-${RUN_NAME:-}}"
if [ -z "$RUN_NAME" ]; then
  usage
  exit 2
fi

case "$RUN_NAME" in
  *[!A-Za-z0-9_-]*)
    echo "Invalid profile name: $RUN_NAME" >&2
    echo "Use only letters, numbers, underscore, and dash." >&2
    exit 2
    ;;
esac

CATKIN_WS="${CATKIN_WS:-$HOME/catkin_ws}"
CONTROL_PKG="$CATKIN_WS/src/ajou_control"
CONFIG_DIR="$CONTROL_PKG/configs/controller_tuning"
RUN_DIR="$CONTROL_PKG/validation_runs"
CONFIG_FILE="${2:-$CONFIG_DIR/$RUN_NAME.yaml}"
RUN_CSV="$RUN_DIR/$RUN_NAME.csv"

if [ ! -f "$CONFIG_FILE" ]; then
  echo "Controller tuning profile not found: $CONFIG_FILE" >&2
  echo "Create it first, for example:" >&2
  echo "  cp $CONFIG_DIR/baseline.yaml $CONFIG_FILE" >&2
  exit 1
fi

ROUTE_LENGTH_M="$(config_value ROUTE_LENGTH_M route_length_m 120.0)"
TARGET_SPEED_MPS="$(config_value TARGET_SPEED_MPS target_speed_mps 2.0)"
VELOCITY_SCALE="$(config_value VELOCITY_SCALE velocity_scale 0.27778)"
STEERING_SIGN="$(config_value STEERING_SIGN steering_sign -1.0)"
MIN_LOOKAHEAD="$(config_value MIN_LOOKAHEAD min_lookahead 7.0)"
MAX_LOOKAHEAD="$(config_value MAX_LOOKAHEAD max_lookahead 16.0)"
LOOKAHEAD_GAIN="$(config_value LOOKAHEAD_GAIN lookahead_gain 1.0)"
MAX_STEER_RAD="$(config_value MAX_STEER_RAD max_steer_rad 0.35)"
MAX_STEER_RATE="$(config_value MAX_STEER_RATE max_steer_rate 0.8)"
ENABLE_RVIZ="$(config_value ENABLE_RVIZ enable_rviz true)"
COMPARE_WITH="${COMPARE_WITH:-$(yaml_value compare_with "$CONFIG_FILE")}"
HISTORY_GLOB="${HISTORY_GLOB:-$(yaml_value history_glob "$CONFIG_FILE")}"

mkdir -p "$RUN_DIR"

if [ -f "$CATKIN_WS/devel/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$CATKIN_WS/devel/setup.bash"
fi

echo "Controller tuning profile: $RUN_NAME"
echo "Config YAML: $CONFIG_FILE"
echo "Validation CSV: $RUN_CSV"
echo "Existing CSV with the same name will be overwritten."
if [ -n "$COMPARE_WITH" ]; then
  echo "Comparing with: $COMPARE_WITH"
fi
if [ -n "$HISTORY_GLOB" ]; then
  echo "History glob: $HISTORY_GLOB"
fi

roslaunch ajou_control mgeo_path_stack.launch \
  route_length_m:="$ROUTE_LENGTH_M" \
  enable_controller:=true \
  enable_validator:=true \
  target_speed_mps:="$TARGET_SPEED_MPS" \
  velocity_scale:="$VELOCITY_SCALE" \
  steering_sign:="$STEERING_SIGN" \
  min_lookahead:="$MIN_LOOKAHEAD" \
  max_lookahead:="$MAX_LOOKAHEAD" \
  lookahead_gain:="$LOOKAHEAD_GAIN" \
  max_steer_rad:="$MAX_STEER_RAD" \
  max_steer_rate:="$MAX_STEER_RATE" \
  validation_csv:="$RUN_CSV" \
  history_csvs:="$COMPARE_WITH" \
  history_glob:="$HISTORY_GLOB" \
  enable_rviz:="$ENABLE_RVIZ"
