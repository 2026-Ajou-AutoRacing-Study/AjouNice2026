#!/usr/bin/env bash
set -euo pipefail

# catkin_add_nosetests supplies nose-style CLI arguments. Translate the small
# supported subset to pytest because this suite uses pytest fixtures. Keep the
# declared Ubuntu NumPy/SciPy stack isolated from unrelated user-site wheels.
export PYTHONNOUSERSITE=1

pytest_args=()
for argument in "$@"; do
  case "$argument" in
    -P|--with-xunit|--process-timeout=*)
      ;;
    --where=*)
      pytest_args+=("${argument#--where=}")
      ;;
    --xunit-file=*)
      pytest_args+=("--junitxml=${argument#--xunit-file=}")
      ;;
    *)
      pytest_args+=("$argument")
      ;;
  esac
done

exec /usr/bin/py.test-3 "${pytest_args[@]}"
