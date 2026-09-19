#!/usr/bin/env bash
set -euo pipefail

root=/home/jihun/Autoware2ros1
bdp="$root/porting/work/autoware_motion_velocity_boundary_departure_prevention_module/candidate-r002-transport-lifecycle-r005-install"
build="$root/porting/work/autoware_motion_velocity_boundary_departure_prevention_module/candidate-r002-transport-lifecycle-r005-build"
prefixes=(
  "$bdp"
  "$root/porting/work/autoware-component-interface-utils-v1-p2-retry006/catkin_ws/devel"
  "$root/porting/work/autoware-deprecated-boundary-departure-checker-v1-p2-001/source-closure-full-001/catkin_ws/install"
  "$root/porting/work/autoware-motion-velocity-planner-v1-p2-001/source-closure-001/install-clean-r003"
  "$root/porting/work/autoware-motion-velocity-planner-common-v1-p2-001/source-closure-002-clean-r003/install"
  "$root/porting/work/autoware-velocity-smoother-v1-p2-001/ros1-runtime-candidate/catkin_ws_r002/install"
  "$root/porting/work/autoware-osqp-interface-v1-p2-001/source-closure/install"
  "$root/porting/work/autoware-qp-interface-v1-p2-001/source-closure/install"
  "$root/porting/work/osqp-v0.6.3-causal-execution-v17-20260831-001/install"
  "$root/porting/work/proxsuite-v0.6.5-provider-001/install"
  "$root/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install"
  "$root/porting/work/autoware-trajectory-v1-p2-001/source-closure-003/install"
  "$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install"
  "$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install"
  "$root/porting/work/autoware-lanelet2-extension-v1-p2-001/source-closure-001/install"
  "$root/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install"
  "$root/porting/work/autoware-object-recognition-utils-v1/candidate-r012/install-r001"
  "$root/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated"
  /opt/ros/noetic
)
for prefix in "${prefixes[@]}" "$build"; do test -d "$prefix"; done
export CMAKE_PREFIX_PATH=$(IFS=':'; printf '%s' "${prefixes[*]}")
export ROS_PACKAGE_PATH=$(IFS=':'; printf '%s/share' "${prefixes[*]}")
libs=()
for prefix in "${prefixes[@]}"; do libs+=("$prefix/lib"); done
libs+=(/opt/ros/noetic/lib/x86_64-linux-gnu)
export LD_LIBRARY_PATH=$(IFS=':'; printf '%s' "${libs[*]}")
export ROS_MASTER_URI=http://127.0.0.1:11551
export ROS_HOME=/tmp/aw149-bdp-clean-r005-roshome
rm -rf "$ROS_HOME"
roscore -p 11551 >/tmp/aw149-bdp-clean-r005-roscore.log 2>&1 &
master_pid=$!
trap 'kill "$master_pid" 2>/dev/null || true; wait "$master_pid" 2>/dev/null || true' EXIT
for _ in $(seq 1 100); do rosparam list >/dev/null 2>&1 && break; sleep 0.1; done
rosparam list >/dev/null
ldd -r "$bdp/lib/libautoware_motion_velocity_boundary_departure_prevention_module.so" | grep -E 'undefined symbol|not found' && exit 1 || true
rospack plugins --attrib=plugin autoware_motion_velocity_planner | grep -F "$bdp/share/autoware_motion_velocity_boundary_departure_prevention_module/plugins.xml"
"$build/devel/lib/autoware_motion_velocity_boundary_departure_prevention_module/bdp_pluginlib_load_oracle"
"$build/devel/lib/autoware_motion_velocity_boundary_departure_prevention_module/bdp_input_gate_oracle"
"$build/devel/lib/autoware_motion_velocity_boundary_departure_prevention_module/bdp_planning_fixture_oracle"
printf 'P3_BDP_CLEAN_R005_PLUGIN_INPUT_POLICY_RUNTIME_PASS tests=3\n'
