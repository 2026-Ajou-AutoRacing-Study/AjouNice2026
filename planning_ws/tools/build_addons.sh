#!/usr/bin/env bash
set -eo pipefail
planning_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${AJOUNICE_ADDON_BUILD_ENV:-}" != 1 ]]; then
  exec env -i PATH=/usr/bin:/bin LANG=C.UTF-8 AJOUNICE_ADDON_BUILD_ENV=1 bash --noprofile --norc "$0" "$@"
fi
source "$planning_root/install_isolated/setup.bash"
repo_root="$(dirname "$planning_root")"
cd "$repo_root/base_ws"
exec catkin_make_isolated --source "$repo_root/base_ws/src/planning_addons" \
  --build "$repo_root/base_ws/build_planning_addons" --devel "$repo_root/base_ws/devel_planning_addons" \
  --install-space "$repo_root/base_ws/install_planning_addons" --install -j4 "$@" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
