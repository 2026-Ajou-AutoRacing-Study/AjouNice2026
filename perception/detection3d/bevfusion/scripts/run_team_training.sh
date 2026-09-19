#!/usr/bin/env bash
set -euo pipefail

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
if [[ -f "$repo_root/.env.bevfusion.local" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$repo_root/.env.bevfusion.local"
  set +a
fi

exec python3 \
  "$repo_root/perception/detection3d/bevfusion/tools/run_team_experiment.py" \
  "$@"
