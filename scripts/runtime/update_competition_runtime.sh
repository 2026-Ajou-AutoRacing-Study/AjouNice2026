#!/usr/bin/env bash
# Update an existing workspace only; never launch nodes or change shell defaults.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MAP_VERSION=95
MAP_REMOTE=()
INSTALL_ARGS=()
# Rebuild shared lane-change interface consumers together, not just its plugin.
PLANNING=(autoware_route_handler autoware_behavior_path_lane_change_module autoware_behavior_path_avoidance_by_lane_change_module autoware_behavior_path_external_request_lane_change_module autoware_behavior_path_planner autoware_motion_velocity_obstacle_cruise_module autoware_path_optimizer autoware_launch)
ASSET_PATHS=(bringup competition/competition_rule_monitor lanelet_map_visualizer localization/localization_manager tracking_prediction_evaluation control/trajectory_batch_runner control/trajectory_generator)
ASSETS=()
usage() {
  echo 'Usage: bash scripts/runtime/update_competition_runtime.sh [--map-version 94|95] [--map-remote NAME] [installer options]'
  echo 'Requires an existing AW149 workspace. Default map is 95 and is verified before building.'
  echo 'Installer options: --existing-python PATH --archive TAR --device cpu|N --models-dir DIR --env-prefix DIR --remote NAME'
  echo '--check, --models-only and --skip-build are rejected; use the underlying installer for partial/check-only work.'
  echo 'Build/install only: no ROS nodes, controller, shell-profile edits or closed-loop approval.'
}
die() { echo "ERROR: $*" >&2; exit 2; }
while (($#)); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --check|--models-only|--skip-build) die "$1 is not a complete runtime update; use the underlying installer" ;;
    --map-version|--map-remote|--existing-python|--archive|--device|--models-dir|--env-prefix|--remote)
      [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || die "Missing value for $1"
      case "$1" in
        --map-version) [[ "$2" == 94 || "$2" == 95 ]] || die 'Map version must be 94 or 95'; MAP_VERSION="$2" ;;
        --map-remote) [[ "$2" =~ ^[A-Za-z0-9_.-]+$ ]] || die 'Invalid map remote name'; MAP_REMOTE=(--remote "$2") ;;
        --device) [[ "$2" == cpu || "$2" =~ ^[0-9]+$ ]] || die 'Device must be cpu or a GPU number'; INSTALL_ARGS+=("$1" "$2") ;;
        *) INSTALL_ARGS+=("$1" "$2") ;;
      esac
      shift 2 ;;
    *) die "Unknown option: $1 (see --help)" ;;
  esac
done
for setup in planning_ws/install_isolated/setup.bash base_ws/install_aw149/setup.bash; do
  [[ -f "$ROOT/$setup" ]] || die "Existing workspace required: missing $setup; use scripts/workspace/setup_ws.sh first"
done
check_cache() {
  /usr/bin/python3 - "$1" "$2" "$3" <<'PY'
import sys
from pathlib import Path
cache, source, prefix = map(Path, sys.argv[1:])
values = {line.split(':', 1)[0]: line.split('=', 1)[1]
          for line in cache.read_text().splitlines()
          if not line.startswith(('#', '//')) and ':' in line and '=' in line}
for key, expected in (('CMAKE_HOME_DIRECTORY', source), ('CMAKE_INSTALL_PREFIX', prefix)):
    if Path(values.get(key, '')).resolve() != expected.resolve():
        raise SystemExit('Refusing foreign/moved CMake cache: ' + str(cache) + ' (' + key + ')')
PY
}
# Validate all existing caches before provisioning or rebuilding anything.
for package in "${PLANNING[@]}" autoware_behavior_velocity_traffic_light_module; do
  cache="$ROOT/planning_ws/build_isolated/$package/CMakeCache.txt"
  if [[ -f "$cache" ]]; then check_cache "$cache" "$ROOT/planning_ws/src/$package" "$ROOT/planning_ws/install_isolated"; fi
done
ALL_CACHED=true
for relative in "${ASSET_PATHS[@]}"; do
  package="${relative##*/}"; ASSETS+=("$package")
  cache="$ROOT/base_ws/build_aw149/$package/CMakeCache.txt"
  if [[ -f "$cache" ]]; then
    check_cache "$cache" "$ROOT/base_ws/src/$relative" "$ROOT/base_ws/install_aw149"
  else
    ALL_CACHED=false
  fi
done
for relative in perception/traffic_light_detection planning_addons/planning_scenario_validator planning_addons/ajounice_planning_integration planning_addons/competition_route_requester perception_object_pipeline; do
  package="${relative##*/}"
  cache="$ROOT/base_ws/build_aw149/$package/CMakeCache.txt"
  if [[ -f "$cache" ]]; then check_cache "$cache" "$ROOT/base_ws/src/$relative" "$ROOT/base_ws/install_aw149"; fi
done
if [[ "$MAP_VERSION" == 94 ]]; then
  /usr/bin/python3 "$ROOT/scripts/map/prepare_kcity_v94.py" "${MAP_REMOTE[@]}"
else
  [[ ${#MAP_REMOTE[@]} == 0 ]] || die 'v95 is supplied separately; --map-remote applies only to v94'
  /usr/bin/python3 "$ROOT/scripts/map/prepare_kcity_v95.py" --check
fi
bash "$ROOT/planning_ws/tools/build.sh" --pkg "${PLANNING[@]}" --make-args -j2 -l2
export AJOUNICE_ROOT="$ROOT"
set +u
source "$ROOT/scripts/workspace/source_aw149.sh"
set -u
if $ALL_CACHED; then
  for package in "${ASSETS[@]}"; do
    cmake --build "$ROOT/base_ws/build_aw149/$package" --target install -- -j2
  done
else
  catkin_make_isolated --install --source "$ROOT/base_ws/src" --build "$ROOT/base_ws/build_aw149" \
    --devel "$ROOT/base_ws/devel_aw149" --install-space "$ROOT/base_ws/install_aw149" \
    --pkg "${ASSETS[@]}" -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 --make-args -j2 -l2
fi
bash "$ROOT/scripts/runtime/install_traffic_light_candidate.sh" "${INSTALL_ARGS[@]}"
# CMake install does not remove files deleted from source. Remove only retired
# v88 requester assets after the replacement installation has succeeded.
for prefix in base_ws/install_aw149 base_ws/install_planning_addons planning_ws/install_isolated; do
  for retired in competition_route_requester/config/competition_route_requester_v88.param.yaml competition_route_requester/config/kcity_v88_competition_route.yaml ajounice_planning_integration/launch/competition_planning_v88.launch; do
    rm -f -- "$ROOT/$prefix/share/$retired"
  done
done
echo '[INSTALLED] Scoped build/install finished. No live nodes were started; driving is not certified.'
printf 'Use separate new terminals with the same explicit map selection:\n  planning map_version:=%s\n  traffic map_version:=%s\n  tracking map_version:=%s\n' "$MAP_VERSION" "$MAP_VERSION" "$MAP_VERSION"
