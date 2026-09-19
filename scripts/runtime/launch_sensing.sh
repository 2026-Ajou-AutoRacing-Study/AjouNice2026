#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$repo_root/scripts/workspace/source_aw149.sh"

roslaunch bringup sensing.launch "$@"
