#!/usr/bin/env bash
# 기본은 계획이다. 실제 재개에는 --run --confirmed-environment sunny15가 필요하다.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$ROOT/base_ws/install_aw149/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311
cd "$ROOT"
exec /usr/bin/python3 -B -u scripts/portable_base5/unified_capture.py \
  --profile "$ROOT/artifacts/evaluation/base5_unified_runtime_261018/sunny15_profile.json" "$@"
