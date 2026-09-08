# ROS1 AW149 planning entry

`launch/planning_ros1.launch` composes the native ROS1 planning stack. Supply
`vehicle_param_path` pointing to native ROS1 vehicle YAML. Map/projector,
localization, perception, operation mode and route requests are external inputs;
this entry does not fabricate them. No vehicle output command is produced here.

The original planning configuration is retained under `config/reference_ros2`
and its native parameter projection under `config/ros1`. Module configuration
preserves the original default ordering: 12 BPP managers, 14 BVP plugins and
9 MVP plugins. BVP experimental mode is the original default; normal mode is
available with `bvp_experimental:=false`.

This is the planning portion of `autoware_launch`, not a port of the separate
ROS2 control/localization/perception stacks, rviz2 or neural-planner model
deployment. Those external stacks are outside the AW149 planning boundary.
Agnocast enabled/ROS2 component execution is not claimed; native ROS1 processes
own their transport and callbacks. Signal outputs may exist internally but
this launch does not connect direction indicators to vehicle control.

The graph includes Mission -> BPP -> BVP -> path smoothing/optimization -> MVP,
parking/costmap branch, scenario selection, velocity smoothing, full Validator,
Surround and velocity-limit selection. Fourteen native processes are started.
Missing mandatory inputs must not yield a final trajectory. Startup and typed
graph acceptance are not an all-missions driving test or AjouNice integration.
