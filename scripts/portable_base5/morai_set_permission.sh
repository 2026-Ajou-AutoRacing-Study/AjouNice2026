#!/usr/bin/env bash
# The launcher expects setPermission.sh beside its executable. No sudo/system edits.
set -euo pipefail
ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ "$ROOT" != "$HOME/MoraiLauncher_Lin" || ! -d "$ROOT/MoraiLauncher_Lin_Data" || -L "$ROOT/MoraiLauncher_Lin_Data" ]]; then
  printf '%s\n' 'Refusing permissions outside the approved HOME/MoraiLauncher_Lin data directory.' >&2
  exit 1
fi
cd -- "$ROOT"
chmod -R a+x -- MoraiLauncher_Lin_Data
