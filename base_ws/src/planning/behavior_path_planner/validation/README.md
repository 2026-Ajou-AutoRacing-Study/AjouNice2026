# Behavior Path Planner Validation Assets

This directory contains files used only for module-level differential testing:

- `launch/`: replay, standalone validation, and ROS 2/ROS 1 comparison launches
- `rviz/`: comparison displays
- `scripts/`: synthetic input and JSON overlay publishers

Runtime planners, shared engines, and competition launches remain in the package's
top-level `scripts/`, `config/`, and `launch/` directories.

Validation assets are installed only when `CATKIN_ENABLE_TESTING=ON`. A production
build with `CATKIN_ENABLE_TESTING=OFF` excludes these scripts, launches, and RViz
configs. Unit tests remain in `test/` and should be run before release.

ROS launch resolves files recursively, so existing commands keep the basename form:

```bash
roslaunch behavior_path_planner bpp_start_planner_comparison.launch ...
```
