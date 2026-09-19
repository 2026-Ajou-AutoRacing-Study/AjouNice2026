#!/usr/bin/env bash
set -euo pipefail

root=/home/jihun/Autoware2ros1
work="$root/porting/work/autoware-behavior-velocity-virtual-traffic-light-module-v1-p2-001/candidate-r028-full-lifecycle"
build="$work/build-clean-r029"
prefixes=(
  "$build/devel" "$work/install-clean-r029"
  "$root/porting/work/tier4-v2x-msgs-v1-p3-clean-r001/install-clean"
  "$root/porting/work/autoware-behavior-velocity-planner-v1-p2-001/source-closure-001/build-clean-r003/devel"
  "$root/porting/work/autoware-behavior-velocity-planner-common-v1-p2-001/source-closure-001/install"
  "$root/porting/work/autoware-trajectory-v1-p2-001/source-closure-003/install"
  "$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install"
  "$root/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install"
  "$root/porting/work/autoware-planning-factor-interface-v1-p2-001/source-closure-003-ros1-abi/install"
  "$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install"
  "$root/porting/work/autoware-velocity-smoother-v1-p2-001/ros1-runtime-candidate/catkin_ws_r002/install"
  "$root/porting/work/autoware-osqp-interface-v1-p2-001/source-closure/install"
  "$root/porting/work/autoware-qp-interface-v1-p2-001/source-closure/install"
  "$root/porting/work/osqp-v0.6.3-causal-execution-v17-20260831-001/install"
  "$root/porting/work/autoware-objects-of-interest-marker-interface-v1-p2-001/attempt-010/install"
  "$root/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install"
  "$root/porting/work/autoware-lanelet2-extension-v1-p2-001/source-closure-001/install"
  "$root/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated"
  "$root/porting/work/autoware-component-interface-specs-v1-p2-retry006/catkin_ws/devel"
)
ld=""; ros_paths=("$root/porting/work/autoware-behavior-velocity-planner-v1-p2-001/source-closure-001/src")
for p in "${prefixes[@]}"; do
  test -d "$p"
  ld="${ld:+$ld:}$p/lib"
  test -d "$p/share" && ros_paths+=("$p/share")
done
export CMAKE_PREFIX_PATH="$(IFS=:; printf '%s' "${prefixes[*]}"):/opt/ros/noetic"
export ROS_PACKAGE_PATH="$(IFS=:; printf '%s' "${ros_paths[*]}"):/opt/ros/noetic/share"
export LD_LIBRARY_PATH="$ld:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu"
port="${AW149_VTL_PORT:-11463}"
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_HOME=/tmp/aw149-vtl-clean-r029-ros-home
export ROS_LOG_DIR=/tmp/aw149-vtl-clean-r029-ros-log
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

roscore -p "$port" >"/tmp/aw149-vtl-clean-r029-roscore.log" 2>&1 &
master_pid=$!
cleanup() { kill "$master_pid" 2>/dev/null || true; wait "$master_pid" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
for _ in $(seq 1 40); do rosparam list >/dev/null 2>&1 && break; sleep 0.1; done
rosparam list >/dev/null
ldd -r "$work/install-clean-r029/lib/libautoware_behavior_velocity_virtual_traffic_light_module.so" | tee /tmp/aw149-vtl-clean-r029-ldd.log
if grep -Eq 'not found|undefined symbol' /tmp/aw149-vtl-clean-r029-ldd.log; then exit 1; fi
rospack plugins --attrib=plugin autoware_behavior_velocity_planner | grep -F "$work/install-clean-r029/share/autoware_behavior_velocity_virtual_traffic_light_module/plugins.xml"
bin="$build/devel/lib/autoware_behavior_velocity_virtual_traffic_light_module"
"$bin/test_vtl_utils"
"$bin/test_vtl_r028_plugin"
"$bin/test_vtl_r028_lifecycle"
"$root/porting/work/autoware-behavior-velocity-planner-v1-p2-001/source-closure-001/build-clean-r003/devel/lib/autoware_behavior_velocity_planner/test_behavior_velocity_planner_experimental_manager" \
  --gtest_filter='ExperimentalBehaviorVelocityPlannerManager.LoadsVirtualTrafficLightRos1V2xPlugin'
rosparam get /rosdistro >/dev/null
test "$(rosnode list)" = "/rosout"
printf 'P3_R030_VTL_SCOPED_MASTER_RUNTIME_PASS tests=utils,plugin,lifecycle,manager\n'
