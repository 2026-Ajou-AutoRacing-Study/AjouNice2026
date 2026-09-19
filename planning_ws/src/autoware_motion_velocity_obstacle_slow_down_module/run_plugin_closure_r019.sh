#!/usr/bin/env bash
# Candidate-only closure probe.  It never reads or writes the frozen AW149 run.
set -euo pipefail

root=/home/jihun/Autoware2ros1
candidate="$root/porting/work/autoware-motion-velocity-obstacle-slow-down-module-v1-p2-001/candidate-r019"
build="$root/porting/work/autoware-motion-velocity-obstacle-slow-down-module-v1-p2-001/candidate-r019-build"

export ROS_HOME=/tmp/aw149-obstacle-slow-down-r019
export CMAKE_PREFIX_PATH="$build/install:$root/porting/work/autoware-motion-velocity-planner-v1-p2-001/source-closure-001/install:$root/porting/work/autoware-motion-velocity-planner-common-v1-p2-001/source-closure-002-build/install:$root/porting/work/autoware-velocity-smoother-v1-p2-001/ros1-runtime-candidate/catkin_ws_r002/install:$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install:$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install:$root/porting/work/autoware-lanelet2-extension-v1-p2-001/source-closure-001/install:/opt/ros/noetic"
export ROS_PACKAGE_PATH="$build/install/share:$candidate:$root/porting/work/autoware-motion-velocity-planner-v1-p2-001/source-closure-001/install/share:$root/porting/work/autoware-motion-velocity-planner-common-v1-p2-001/source-closure-002-build/install/share:$root/porting/work/autoware-velocity-smoother-v1-p2-001/ros1-runtime-candidate/catkin_ws_r002/install/share:$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install/share:$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install/share:$root/porting/work/autoware-lanelet2-extension-v1-p2-001/source-closure-001/install/share:$root/porting/work/autoware-object-recognition-utils-v1/candidate-r012/install-r001/share:$root/porting/work/autoware-vehicle-info-utils-v1-p2-001/retry006/catkin_ws/install_isolated/share:/home/jihun/AjouNice2026/base_ws/devel/share:/opt/ros/noetic/share"
export LD_LIBRARY_PATH="$build/devel/lib:$build/install/lib:$root/porting/work/autoware-motion-velocity-planner-common-v1-p2-001/source-closure-002-build/install/lib:$root/porting/work/autoware-velocity-smoother-v1-p2-001/ros1-runtime-candidate/catkin_ws_r002/install/lib:$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install/lib:$root/porting/work/autoware-lanelet2-extension-v1-p2-001/source-closure-001/install/lib:$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install/lib:$root/porting/work/autoware-osqp-interface-v1-p2-001/source-closure/install/lib:$root/porting/work/autoware-qp-interface-v1-p2-001/source-closure/install/lib:$root/porting/work/osqp-v0.6.3-causal-execution-v17-20260831-001/install/lib:$root/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install/lib:$root/porting/work/autoware-object-recognition-utils-v1/candidate-r012/install-r001/lib:$root/porting/work/autoware-objects-of-interest-marker-interface-v1-p2-001/attempt-010/install/lib:$root/porting/work/autoware-signal-processing-v1-actual-render-r003/source-closure-001/install/lib:$root/porting/work/autoware-planning-topic-converter-v1-p2-001/source-closure-001/install/lib:$root/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install/lib:$root/porting/work/autoware-trajectory-v1-p2-001/source-closure-003/install/lib:$root/porting/work/autoware-vehicle-info-utils-v1-p2-001/retry006/catkin_ws/install_isolated/lib:/home/jihun/AjouNice2026/base_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu"

if [[ "${AW149_CLOSURE_ENV_ONLY:-}" == "1" ]]; then
  return 0 2>/dev/null || exit 0
fi

library="$build/install/lib/libautoware_motion_velocity_obstacle_slow_down_module.so"
if ldd -r "$library" | rg -q 'not found|undefined symbol'; then
  echo OBS_SLOW_DOWN_R019_LDD_CLOSURE_FAIL
  ldd -r "$library" | rg 'not found|undefined symbol'
  exit 1
fi
echo OBS_SLOW_DOWN_R019_LDD_CLOSURE_PASS
echo OBS_SLOW_DOWN_R019_PLUGIN_EXPORTS_BEGIN
rospack plugins --attrib=plugin autoware_motion_velocity_planner
echo OBS_SLOW_DOWN_R019_PLUGIN_EXPORTS_END
exec "$build/devel/lib/autoware_motion_velocity_obstacle_slow_down_module/test_obstacle_slow_down_plugin"
