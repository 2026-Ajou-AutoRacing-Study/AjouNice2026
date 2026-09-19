#!/usr/bin/env bash
# Candidate-only DynamicObstacleStop closure probe; imports no frozen AW149 state.
set -euo pipefail
root=/home/jihun/Autoware2ros1
AW149_CLOSURE_ENV_ONLY=1 source "$root/porting/work/autoware-motion-velocity-obstacle-slow-down-module-v1-p2-001/candidate-r019/run_plugin_closure_r019.sh"

build="$root/porting/work/autoware-motion-velocity-dynamic-obstacle-stop-module-v1-p2-001/candidate-r026-diagnostic-build"
install="$root/porting/work/autoware-motion-velocity-dynamic-obstacle-stop-module-v1-p2-001/candidate-r026-diagnostic-install"
common="$root/porting/work/autoware-motion-velocity-planner-common-v1-p2-001/source-closure-001/install"
vehicle="$root/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated"
export ROS_HOME=/tmp/aw149-dynamic-obstacle-stop-r026
export CMAKE_PREFIX_PATH="$install:$common:$vehicle:$CMAKE_PREFIX_PATH"
export ROS_PACKAGE_PATH="$install/share:$vehicle/share:$root/porting/work/autoware-motion-velocity-planner-v1-p2-001/source-closure-001/install/share:$ROS_PACKAGE_PATH"
export LD_LIBRARY_PATH="$build/devel/lib:$install/lib:$common/lib:$vehicle/lib:$LD_LIBRARY_PATH"

library="$install/lib/libautoware_motion_velocity_dynamic_obstacle_stop_module.so"
if ldd -r "$library" | rg -q 'not found|undefined symbol'; then
  echo DYNAMIC_OBSTACLE_STOP_R026_LDD_CLOSURE_FAIL
  ldd -r "$library" | rg 'not found|undefined symbol'
  exit 1
fi
echo DYNAMIC_OBSTACLE_STOP_R026_LDD_CLOSURE_PASS
rospack plugins --attrib=plugin autoware_motion_velocity_planner
exec "$build/devel/lib/autoware_motion_velocity_dynamic_obstacle_stop_module/verify_dynamic_obstacle_stop_plugin"
