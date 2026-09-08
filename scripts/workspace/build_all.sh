#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export AJOUNICE_ROOT="$REPO_ROOT"

source /opt/ros/noetic/setup.bash

if [[ ! -f "$REPO_ROOT/planning_ws/install_isolated/setup.bash" ]]; then
  "$REPO_ROOT/planning_ws/tools/build.sh" --providers
fi

source "$REPO_ROOT/planning_ws/install_isolated/setup.bash"

for required_submodule in \
  "$REPO_ROOT/base_ws/src/ekf-multi-object-tracking" \
  "$REPO_ROOT/third_party2/livox_sdk"; do
  if ! git -C "$required_submodule" rev-parse --verify HEAD >/dev/null 2>&1; then
    echo "ERROR: initialize submodules first: git submodule update --init --recursive" >&2
    echo "Missing: $required_submodule" >&2
    exit 1
  fi
done

# livox_ros_driver otherwise clones an unpinned SDK HEAD during CMake. Build the
# repository-pinned SDK first and force the driver to consume that exact archive.
LIVOX_SDK_BUILD="$REPO_ROOT/build_external/livox_sdk"
LIVOX_SDK_INSTALL="$REPO_ROOT/build_external/livox_sdk_install"
LIVOX_SDK_LIBRARY="$LIVOX_SDK_INSTALL/lib/liblivox_sdk_static.a"
if [[ ! -f "$LIVOX_SDK_LIBRARY" ]]; then
  cmake -S "$REPO_ROOT/third_party2/livox_sdk" -B "$LIVOX_SDK_BUILD" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$LIVOX_SDK_INSTALL"
  cmake --build "$LIVOX_SDK_BUILD" --target install -- -j4
fi
export CPATH="$LIVOX_SDK_INSTALL/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$LIVOX_SDK_INSTALL/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"

cd "$REPO_ROOT/base_ws"
catkin_make_isolated --install \
  --source "$REPO_ROOT/base_ws/src" \
  --build "$REPO_ROOT/base_ws/build_aw149" \
  --devel "$REPO_ROOT/base_ws/devel_aw149" \
  --install-space "$REPO_ROOT/base_ws/install_aw149" \
  -DLIVOX_SDK_LIBRARY="$LIVOX_SDK_LIBRARY" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --make-args -j4 -l4

# The pinned upstream FAST_LIO revision builds the runtime but declares no
# catkin install rules. Keep the submodule immutable and complete the supported
# install overlay from the built artifact and its runtime assets here.
FAST_LIO_DEVEL="$REPO_ROOT/base_ws/devel_aw149/fast_lio"
FAST_LIO_SOURCE="$REPO_ROOT/base_ws/src/localization/fast_lio"
FAST_LIO_INSTALL="$REPO_ROOT/base_ws/install_aw149"
install -D -m 0755 \
  "$FAST_LIO_DEVEL/lib/fast_lio/fastlio_mapping" \
  "$FAST_LIO_INSTALL/lib/fast_lio/fastlio_mapping"
for asset in launch config rviz_cfg; do
  cmake -E copy_directory \
    "$FAST_LIO_SOURCE/$asset" \
    "$FAST_LIO_INSTALL/share/fast_lio/$asset"
done

# A build is not portable if launch/config assets exist only in the source
# tree. Audit every base package after installation so another checkout fails
# here instead of later at roslaunch time.
missing_assets=0
while IFS= read -r package_xml; do
  package_dir="${package_xml%/package.xml}"
  package_name="$(sed -n 's:.*<name>\([^<]*\)</name>.*:\1:p' "$package_xml" | head -1)"
  [[ -n "$package_name" ]] || continue
  for asset in launch config rviz rviz_cfg; do
    [[ -d "$package_dir/$asset" ]] || continue
    while IFS= read -r source_file; do
      relative_path="${source_file#"$package_dir/"}"
      installed_file="$FAST_LIO_INSTALL/share/$package_name/$relative_path"
      if [[ ! -e "$installed_file" ]]; then
        echo "ERROR: missing installed runtime asset: $source_file" >&2
        missing_assets=$((missing_assets + 1))
      fi
    done < <(find "$package_dir/$asset" -type f | sort)
  done
done < <(find "$REPO_ROOT/base_ws/src" -name package.xml -type f | sort)
if (( missing_assets != 0 )); then
  echo "ERROR: $missing_assets base runtime assets were not installed." >&2
  exit 1
fi

echo "Built planning underlay and base overlay."
echo "Run: source $REPO_ROOT/base_ws/install_aw149/setup.bash"
