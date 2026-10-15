#!/usr/bin/env bash
# Docker/Compose/Toolkit + Drive image/E25 setup. ROS build/map are separate.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec /usr/bin/python3 "$ROOT/base_ws/src/planning_addons/planning_scenario_validator/scripts/setup_bevfusion_inference.py" "$@"
