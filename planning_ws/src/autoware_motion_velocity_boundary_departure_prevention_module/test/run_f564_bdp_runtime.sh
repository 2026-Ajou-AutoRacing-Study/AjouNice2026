#!/usr/bin/env bash
# Candidate-only BDP f564 runtime closure. It owns only this private master.
set -euo pipefail

root="${AW149_WORKSPACE_ROOT:-/home/jihun/Autoware2ros1}"
base=/home/jihun/AjouNice2026/base_ws/devel
bdp="$root/porting/work/autoware_motion_velocity_boundary_departure_prevention_module/candidate-abi-f564-r004-install"
build="$root/porting/work/autoware_motion_velocity_boundary_departure_prevention_module/candidate-abi-f564-r004-build"
mvp="$root/porting/work/autoware-motion-velocity-planner-v1-p2-001/candidate-abi-f564-r003-install"
common="$root/porting/work/autoware-motion-velocity-planner-common-v1-p2-001/candidate-abi-f564-r001-install"
smoother="$root/porting/work/autoware-velocity-smoother-v1-p2-001/ros1-runtime-candidate/candidate-abi-f564-r001-install"
oru="$root/porting/work/autoware-object-recognition-utils-v1/candidate-r016/install-r016"
motion_utils="$root/porting/work/autoware-motion-utils-v1-p2-001/candidate-abi-f564-r002-install"
bdc="$root/porting/work/autoware-deprecated-boundary-departure-checker-v1-p2-001/source-closure-full-001/catkin_ws/install"

for prefix in "$bdp" "$build" "$mvp" "$common" "$smoother" "$oru" "$motion_utils" "$bdc"; do
  [[ -d "$prefix" ]]
done
source "$base/setup.bash"
export ROS_PACKAGE_PATH="$bdp/share:$mvp/share:$common/share:$smoother/share:$oru/share:$motion_utils/share:$bdc/share:/home/jihun/AjouNice2026/base_ws/src/msgs:/home/jihun/AjouNice2026/base_ws/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export CMAKE_PREFIX_PATH="$bdp:$mvp:$common:$smoother:$oru:$motion_utils:$bdc${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
provider_libs=(
  "$bdp/lib" "$mvp/lib" "$common/lib" "$smoother/lib" "$oru/lib" "$motion_utils/lib" "$bdc/lib"
  "$root/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install/lib"
  "$root/porting/work/autoware-lanelet2-extension-v1-p2-001/source-closure-001/install/lib"
  "$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install/lib"
  "$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install/lib"
  "$root/porting/work/autoware-trajectory-v1-p2-001/source-closure-003/install/lib"
  "$root/porting/work/autoware-osqp-interface-v1-p2-001/source-closure/install/lib"
  "$root/porting/work/autoware-qp-interface-v1-p2-001/source-closure/install/lib"
  "$root/porting/work/osqp-v0.6.3-causal-execution-v17-20260831-001/install/lib"
  "$root/porting/work/proxsuite-v0.6.5-provider-001/install/lib"
  "$root/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated/lib"
  /home/jihun/AjouNice2026/base_ws/devel/lib /opt/ros/noetic/lib /opt/ros/noetic/lib/x86_64-linux-gnu
)
IFS=:
export LD_LIBRARY_PATH="${provider_libs[*]}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
unset IFS

export ROS_MASTER_URI=http://127.0.0.1:11550
roscore -p 11550 >/tmp/aw149-bdp-f564-roscore.log 2>&1 &
master_pid=$!
trap 'kill "$master_pid" 2>/dev/null || true; wait "$master_pid" 2>/dev/null || true' EXIT
for _ in $(seq 1 100); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam list >/dev/null
rospack profile >/dev/null
ldd -r "$bdp/lib/libautoware_motion_velocity_boundary_departure_prevention_module.so" | grep -E 'undefined symbol|not found' && exit 1 || true
rospack plugins --attrib=plugin autoware_motion_velocity_planner | grep -F "$bdp/share/autoware_motion_velocity_boundary_departure_prevention_module/plugins.xml"
"$build/devel/lib/autoware_motion_velocity_boundary_departure_prevention_module/bdp_pluginlib_load_oracle"
"$build/devel/lib/autoware_motion_velocity_boundary_departure_prevention_module/bdp_input_gate_oracle"
"$build/devel/lib/autoware_motion_velocity_boundary_departure_prevention_module/bdp_planning_fixture_oracle"
printf 'F564_BDP_PLUGINLIB_INPUT_AND_PLANNING_RUNTIME_PASS\n'
