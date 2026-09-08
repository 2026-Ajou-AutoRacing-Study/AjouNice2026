#!/usr/bin/env bash
# Source only from a clean Noetic shell, never on top of base_ws/devel.
planning_env_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export AJOUNICE_ROOT="$(dirname "$planning_env_root")"
source "$planning_env_root/install_isolated/setup.bash"
if [[ -f "$AJOUNICE_ROOT/base_ws/install_planning_addons/setup.bash" ]]; then
  source "$AJOUNICE_ROOT/base_ws/install_planning_addons/setup.bash"
fi
unset planning_env_root
