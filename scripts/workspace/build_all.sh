#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export AJOUNICE_ROOT="$REPO_ROOT"

clean_base=false
if [[ "${1:-}" == "--clean" ]]; then
  clean_base=true
  shift
fi
if (( $# != 0 )); then
  echo "Usage: $0 [--clean]" >&2
  exit 2
fi

# Verify the team-Drive v99 asset before cleaning any installed workspace.
/usr/bin/python3 "$REPO_ROOT/scripts/map/prepare_kcity_v99.py" --check

# ROS-generated hooks read unset ROS_DISTRO/ROS_MASTER_URI in a fresh shell.
# Keep error checking, but suspend nounset while sourcing ROS setup scripts.
set +u
source /opt/ros/noetic/setup.bash
set -u

PLANNING_INSTALL="$REPO_ROOT/planning_ws/install_isolated"
PLANNING_COMPLETE_MARKER="$PLANNING_INSTALL/.aw149_planning_build_complete"
if [[ ! -f "$PLANNING_INSTALL/setup.bash" || ! -f "$PLANNING_COMPLETE_MARKER" ]]; then
  # catkin_make_isolated writes setup.bash after its first package.  Do not
  # mistake that partial install for a usable planning underlay after a failed
  # build; only this wrapper records a successful full build.
  rm -f "$PLANNING_COMPLETE_MARKER"
  "$REPO_ROOT/planning_ws/tools/build.sh" --providers
  touch "$PLANNING_COMPLETE_MARKER"
fi

set +u
source "$PLANNING_INSTALL/setup.bash"
set -u

if [[ "$clean_base" == "true" ]]; then
  for generated_dir in \
    "$REPO_ROOT/base_ws/build_aw149" \
    "$REPO_ROOT/base_ws/devel_aw149" \
    "$REPO_ROOT/base_ws/install_aw149"; do
    case "$generated_dir" in
      "$REPO_ROOT"/base_ws/build_aw149|"$REPO_ROOT"/base_ws/devel_aw149|"$REPO_ROOT"/base_ws/install_aw149)
        cmake -E remove_directory "$generated_dir"
        ;;
      *)
        echo "ERROR: refusing to clean unexpected path: $generated_dir" >&2
        exit 1
        ;;
    esac
  done
fi

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
# The pinned FAST_LIO packages do not declare that their mapping targets depend
# on the generated Pose6D header. A clean parallel default build can therefore
# compile laserMapping.cpp before Pose6D.h exists. Keep the upstream submodules
# immutable: build the other packages first, configure each package, explicitly
# generate its C++ message header, then let catkin build the complete package.
catkin_make_isolated --install \
  --source "$REPO_ROOT/base_ws/src" \
  --build "$REPO_ROOT/base_ws/build_aw149" \
  --devel "$REPO_ROOT/base_ws/devel_aw149" \
  --install-space "$REPO_ROOT/base_ws/install_aw149" \
  --ignore-pkg fast_lio fast_lio_msf \
  -DLIVOX_SDK_LIBRARY="$LIVOX_SDK_LIBRARY" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --make-args -j4 -l4

FAST_LIO_DEVEL="$REPO_ROOT/base_ws/devel_aw149/fast_lio"
FAST_LIO_SOURCE="$REPO_ROOT/base_ws/src/localization/fast_lio"
FAST_LIO_BUILD="$REPO_ROOT/base_ws/build_aw149/fast_lio"
FAST_LIO_INSTALL="$REPO_ROOT/base_ws/install_aw149"
mkdir -p "$FAST_LIO_BUILD"
"$FAST_LIO_INSTALL/env.sh" cmake \
  -S "$FAST_LIO_SOURCE" \
  -B "$FAST_LIO_BUILD" \
  -DCATKIN_DEVEL_PREFIX="$FAST_LIO_DEVEL" \
  -DCMAKE_INSTALL_PREFIX="$FAST_LIO_INSTALL" \
  -DLIVOX_SDK_LIBRARY="$LIVOX_SDK_LIBRARY" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
"$FAST_LIO_INSTALL/env.sh" cmake --build "$FAST_LIO_BUILD" \
  --target fast_lio_generate_messages_cpp -- -j4 -l4
catkin_make_isolated --install \
  --source "$REPO_ROOT/base_ws/src" \
  --build "$REPO_ROOT/base_ws/build_aw149" \
  --devel "$REPO_ROOT/base_ws/devel_aw149" \
  --install-space "$REPO_ROOT/base_ws/install_aw149" \
  --pkg fast_lio \
  -DLIVOX_SDK_LIBRARY="$LIVOX_SDK_LIBRARY" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --make-args -j4 -l4

# The pinned upstream FAST_LIO revision builds the runtime but declares no
# catkin install rules. Complete the supported install overlay from the built
# artifact and its runtime assets without changing the submodule checkout.
install -D -m 0755 \
  "$FAST_LIO_DEVEL/lib/fast_lio/fastlio_mapping" \
  "$FAST_LIO_INSTALL/lib/fast_lio/fastlio_mapping"
for asset in launch config rviz_cfg; do
  cmake -E copy_directory \
    "$FAST_LIO_SOURCE/$asset" \
    "$FAST_LIO_INSTALL/share/fast_lio/$asset"
done

# fast_lio_msf has the same generated-header ordering flaw and no install
# rules. Build it after generating Pose6D.h, then complete the install overlay
# from its runtime artifact and assets without modifying the upstream submodule.
FAST_LIO_MSF_SOURCE="$REPO_ROOT/base_ws/src/localization/fast_lio_multi_sensor_fusion"
FAST_LIO_MSF_DEVEL="$REPO_ROOT/base_ws/devel_aw149/fast_lio_msf"
FAST_LIO_MSF_BUILD="$REPO_ROOT/base_ws/build_aw149/fast_lio_msf"
mkdir -p "$FAST_LIO_MSF_BUILD"
"$FAST_LIO_INSTALL/env.sh" cmake \
  -S "$FAST_LIO_MSF_SOURCE" \
  -B "$FAST_LIO_MSF_BUILD" \
  -DCATKIN_DEVEL_PREFIX="$FAST_LIO_MSF_DEVEL" \
  -DCMAKE_INSTALL_PREFIX="$FAST_LIO_INSTALL" \
  -DLIVOX_SDK_LIBRARY="$LIVOX_SDK_LIBRARY" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
"$FAST_LIO_INSTALL/env.sh" cmake --build "$FAST_LIO_MSF_BUILD" \
  --target fast_lio_msf_generate_messages_cpp -- -j4 -l4
catkin_make_isolated --install \
  --source "$REPO_ROOT/base_ws/src" \
  --build "$REPO_ROOT/base_ws/build_aw149" \
  --devel "$REPO_ROOT/base_ws/devel_aw149" \
  --install-space "$REPO_ROOT/base_ws/install_aw149" \
  --pkg fast_lio_msf \
  -DLIVOX_SDK_LIBRARY="$LIVOX_SDK_LIBRARY" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --make-args -j4 -l4
install -D -m 0755 \
  "$FAST_LIO_MSF_DEVEL/lib/fast_lio_msf/fastlio_msf_mapping" \
  "$FAST_LIO_INSTALL/lib/fast_lio_msf/fastlio_msf_mapping"
for asset in launch config rviz_cfg; do
  cmake -E copy_directory \
    "$FAST_LIO_MSF_SOURCE/$asset" \
    "$FAST_LIO_INSTALL/share/fast_lio_msf/$asset"
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
      legacy_flat_file="$FAST_LIO_INSTALL/share/$package_name/$(basename "$source_file")"
      # A package-level README (or another package-root source file) may be
      # installed here intentionally, independently of nested config assets.
      if [[ "$legacy_flat_file" != "$installed_file" && -e "$legacy_flat_file" \
            && ! -f "$package_dir/$(basename "$source_file")" ]]; then
        echo "ERROR: ambiguous legacy flat install asset: $legacy_flat_file" >&2
        missing_assets=$((missing_assets + 1))
      fi
    done < <(find "$package_dir/$asset" -type f | sort)
  done
done < <(find "$REPO_ROOT/base_ws/src" -name package.xml -type f | sort)
if (( missing_assets != 0 )); then
  echo "ERROR: $missing_assets base runtime install asset errors." >&2
  echo "Retry with: $0 --clean" >&2
  exit 1
fi

echo "Built planning underlay and base overlay."
echo "Run: source $REPO_ROOT/base_ws/install_aw149/setup.bash"
