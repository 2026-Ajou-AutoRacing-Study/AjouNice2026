#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
candidate_prefix="$root_dir/porting/work/autoware-behavior-velocity-virtual-traffic-light-module-v1-p2-001/candidate-r028-install"
build_dir="$root_dir/porting/work/autoware-behavior-velocity-virtual-traffic-light-module-v1-p2-001/candidate-r028-full-lifecycle-build"
bvp_dir="$root_dir/porting/work/autoware-behavior-velocity-planner-v1-p2-001/source-closure-001/src/autoware_behavior_velocity_planner"

provider_libs=(
  "$root_dir/porting/work/autoware-behavior-velocity-virtual-traffic-light-module-v1-p2-001/candidate-r028-install/lib"
  "$root_dir/porting/work/autoware-behavior-velocity-planner-common-v1-p2-001/source-closure-001/install/lib"
  "$root_dir/porting/work/autoware-trajectory-v1-p2-001/source-closure-003/install/lib"
  "$root_dir/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install/lib"
  "$root_dir/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install/lib"
  "$root_dir/porting/work/autoware-planning-factor-interface-v1-p2-001/source-closure-003-ros1-abi/install/lib"
  "$root_dir/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install/lib"
  "$root_dir/porting/work/autoware-velocity-smoother-v1-p2-001/ros1-runtime-candidate/catkin_ws_r002/install/lib"
  "$root_dir/porting/work/autoware-osqp-interface-v1-p2-001/source-closure/install/lib"
  "$root_dir/porting/work/autoware-qp-interface-v1-p2-001/source-closure/install/lib"
  "$root_dir/porting/work/osqp-v0.6.3-causal-execution-v17-20260831-001/install/lib"
  "$root_dir/porting/work/autoware-objects-of-interest-marker-interface-v1-p2-001/attempt-010/install/lib"
  "$root_dir/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install/lib"
  "$root_dir/porting/work/autoware-lanelet2-extension-v1-p2-001/source-closure-001/install/lib"
  "$root_dir/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated/lib"
)

export ROS_HOME=/tmp/aw149-vtl-r028-ros-home
mkdir -p "$ROS_HOME"
export CMAKE_PREFIX_PATH="$candidate_prefix:${CMAKE_PREFIX_PATH}"
export ROS_PACKAGE_PATH="$candidate_prefix/share:$bvp_dir:${ROS_PACKAGE_PATH}"
export LD_LIBRARY_PATH="$(IFS=:; echo "${provider_libs[*]}")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11488}"

# Refresh the installed candidate from this exact build.  A caller-owned
# foreground master is required; do not hide a private roscore in this gate.
cmake --build "$build_dir" --target test_vtl_utils test_vtl_r028_plugin test_vtl_r028_lifecycle -j2
cmake --install "$build_dir" >/tmp/aw149-vtl-r028-install.log
source "$candidate_prefix/setup.bash"
export ROS_PACKAGE_PATH="$candidate_prefix/share:$bvp_dir:${ROS_PACKAGE_PATH}"
export LD_LIBRARY_PATH="$(IFS=:; echo "${provider_libs[*]}")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if ! rosparam list >/dev/null 2>&1; then
  echo "VTL_R028_NO_FOREGROUND_ROSCORE uri=$ROS_MASTER_URI" >&2
  exit 2
fi
ldd -r "$candidate_prefix/lib/libautoware_behavior_velocity_virtual_traffic_light_module.so" | tee /tmp/aw149-vtl-r028-ldd.txt
if rg -q 'not found|undefined symbol' /tmp/aw149-vtl-r028-ldd.txt; then
  echo 'VTL_R028_LDD_CLOSURE_FAIL' >&2
  exit 3
fi
rospack plugins --attrib=plugin autoware_behavior_velocity_planner | grep -F 'autoware_behavior_velocity_virtual_traffic_light_module/plugins.xml'
for test_name in test_vtl_utils test_vtl_r028_plugin test_vtl_r028_lifecycle; do
  timeout 45 "$build_dir/devel/lib/autoware_behavior_velocity_virtual_traffic_light_module/$test_name" --gtest_color=no
done
printf 'VTL_R028_FOREGROUND_RUNTIME_PASS tests=utils,plugin,lifecycle\n'
