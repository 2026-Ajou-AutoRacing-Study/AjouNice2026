#!/usr/bin/env bash

set -Eeuo pipefail

CAPTURE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AJOUNICE_ROOT="${AJOUNICE_ROOT:-$(cd "$CAPTURE_DIR/../../.." && pwd)}"
export AJOUNICE_ROOT

source_capture_workspace() {
  source /opt/ros/noetic/setup.bash
  local setup="$AJOUNICE_ROOT/base_ws/devel/setup.bash"
  if [[ ! -f "$setup" ]]; then
    echo "ERROR: minimal capture workspace is not built: $setup" >&2
    echo "Run: dataset/morai_od/human_eval_capture/setup_capture_environment.sh" >&2
    return 2
  fi
  source "$setup"
}

plan_file="$CAPTURE_DIR/capture_plan.json"
scenario_source_dir="$AJOUNICE_ROOT/configs/scenario/human_driving_evaluation"
runtime_dir="${XDG_RUNTIME_DIR:-/tmp}/ajounice_human_eval_${UID}"
mkdir -p "$runtime_dir"
