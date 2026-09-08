#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

git -C "$REPO_ROOT" submodule update --init --recursive
"$REPO_ROOT/scripts/workspace/build_all.sh"

echo "Workspace initialized."
