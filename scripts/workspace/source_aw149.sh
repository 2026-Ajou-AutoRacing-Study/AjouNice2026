#!/usr/bin/env bash
# Source the single supported AW149 runtime overlay.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "ERROR: source this file instead of executing it." >&2
  exit 2
fi

_aw149_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export AJOUNICE_ROOT="${AJOUNICE_ROOT:-$_aw149_repo_root}"

if [[ "$AJOUNICE_ROOT" != "$_aw149_repo_root" ]]; then
  echo "ERROR: AJOUNICE_ROOT points to '$AJOUNICE_ROOT', expected '$_aw149_repo_root'." >&2
  return 1
fi

source /opt/ros/noetic/setup.bash

_aw149_setup="$AJOUNICE_ROOT/base_ws/install_aw149/setup.bash"
if [[ ! -f "$_aw149_setup" ]]; then
  echo "ERROR: AW149 overlay is not installed: $_aw149_setup" >&2
  echo "Run: $AJOUNICE_ROOT/scripts/workspace/setup_ws.sh" >&2
  return 1
fi

source "$_aw149_setup"
unset _aw149_setup _aw149_repo_root
