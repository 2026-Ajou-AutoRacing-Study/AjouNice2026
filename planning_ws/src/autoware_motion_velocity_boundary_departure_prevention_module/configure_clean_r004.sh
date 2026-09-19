#!/usr/bin/env bash
set -euo pipefail

root=/home/jihun/Autoware2ros1
source_root="$root/porting/work/autoware_motion_velocity_boundary_departure_prevention_module/candidate-r002-transport-lifecycle"
build="$root/porting/work/autoware_motion_velocity_boundary_departure_prevention_module/candidate-r002-transport-lifecycle-r004-build"
install="$root/porting/work/autoware_motion_velocity_boundary_departure_prevention_module/candidate-r002-transport-lifecycle-r004-install"

# BDP consumes MVP-common data, so it must use the same clean ABI closure as
# the validated MVP and never inherit the active AjouNice workspace.
prefixes=(
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
for prefix in "${prefixes[@]}"; do test -d "$prefix"; done
cmake_prefix=$(IFS=';'; printf '%s' "${prefixes[*]}")
export CMAKE_PREFIX_PATH=$(IFS=':'; printf '%s' "${prefixes[*]}")
unset ROS_PACKAGE_PATH
cmake -S "$source_root" -B "$build" -DCMAKE_PREFIX_PATH="$cmake_prefix" -DCMAKE_INSTALL_PREFIX="$install"
