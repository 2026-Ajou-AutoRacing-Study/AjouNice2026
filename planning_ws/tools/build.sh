#!/usr/bin/env bash
set -eo pipefail
planning_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${AJOUNICE_PLANNING_BUILD_ENV:-}" != 1 ]]; then
  exec env -i PATH=/usr/bin:/bin LANG=C.UTF-8 AJOUNICE_PLANNING_BUILD_ENV=1 \
    bash --noprofile --norc "$0" "$@"
fi
source /opt/ros/noetic/setup.bash
mkdir -p "$planning_root/logs"
if [[ "${1:-}" == --providers ]]; then
  shift
  python3 "$planning_root/tools/prepare_osqp.py"
  python3 "$planning_root/tools/prepare_osqp.py" --provider proxsuite
  python3 "$planning_root/tools/prepare_acados.py"
fi
export CMAKE_PREFIX_PATH="$planning_root/external/install:$planning_root/external/acados/install:$CMAKE_PREFIX_PATH"
cd "$planning_root"
exec catkin_make_isolated --install -j4 "$@" --cmake-args \
  '-Uautoware_*_DIR' '-Utier4_*_DIR' '-Ubuiltin_interfaces_DIR' \
  '-Uunique_identifier_msgs_DIR' '-Utl_expected_DIR' '-Ulib' \
  "-DCMAKE_PREFIX_PATH=$planning_root/install_isolated;$planning_root/external/install;$planning_root/external/acados/install;/opt/ros/noetic" \
  "-DACADOS_ROOT=$planning_root/external/acados/install" \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
