# Motion Velocity Planner Validation Materials

## Automated case runner

Build the package and run the validation launch:

```bash
source /opt/ros/noetic/setup.bash
cd ~/AjouNice2026/base_ws
catkin build motion_velocity_planner
source devel/setup.bash
roslaunch motion_velocity_planner validate_motion_velocity_planner.launch
```

If message generation fails with `Unable to find either executable 'empy'` or `module 'em' has no attribute 'RAW_OPT'`, install the ROS Noetic-compatible Empy version:

```bash
python3 -m pip install --user --force-reinstall 'empy==3.3.4'
```

The launch starts `motion_velocity_planner`, publishes synthetic path/odometry/object cases, subscribes to `/motion_velocity_planner/output/path`, and writes:

- `~/AjouNice2026/artifacts/mvp_validation/mvp_validation_results.json`
- `~/AjouNice2026/artifacts/mvp_validation/mvp_validation_results.csv`

Default cases live in:

```text
motion_velocity_planner/config/validation/mvp_cases.yaml
```

Each case can set `object_x`, `object_y`, `object_speed`, `path_speed`, `ego_speed`, and `expected_stop_x`. Use `expected_stop_x: null` when the object should not affect the output path.

## Docker and planning simulator notes

This runner only depends on ROS topics, so the same case file can be used in either the local ROS workspace or a Docker-based Autoware planning simulator session.

Current local Docker status:

```text
Docker CLI is installed, but the current user is not in the docker group.
```

Fix once from the host shell, then log out and back in:

```bash
sudo usermod -aG docker "$USER"
```

In a planning simulator session, keep the simulator publishing the real route/path/perception topics and run the runner against the MVP output topic you want to compare. For synthetic MVP-only checks, use `validate_motion_velocity_planner.launch`; for simulator comparison, run only the runner and set the topic arguments:

```bash
rosrun motion_velocity_planner run_motion_velocity_planner_cases.py \
  --case-file /path/to/mvp_cases.yaml \
  --input-path-topic /behavior_path_planner/output/path \
  --objects-topic /perception/objects \
  --odometry-topic /odometry \
  --output-path-topic /motion_velocity_planner/output/path
```

## End-to-end velocity profile

The following values are based on the executed 30 m obstacle integration test:

- Input path: 1 m spacing, 8 m/s initial velocity
- Planner base velocity: 5 m/s
- Static obstacle: 30 m, 0 m/s
- Dynamic obstacle: 30 m, 2 m/s
- Observed output: both cases stop at 27 m

The overlapping profiles are intentional evidence of the current integration gap: the integrated node still reads `ObjectClassification.confidence`, whereas the ROS message provides `probability`. As a result, its static/dynamic classification branches are inactive and the generic slow-down policy determines the stop point.

## RViz capture guide

Use this layout to produce the companion RViz screenshot.

| Display | Topic | Suggested style |
|---|---|---|
| Input path | `/behavior_path_planner/output/path` | Thin blue line |
| Final path | `/motion_velocity_planner/output/path` | Thick green-to-red line by point velocity |
| Obstacles | `/perception/objects` | Orange bounding boxes |
| Ego pose | `/odometry` | Blue vehicle arrow |

For the validation screenshot, place a 30 m object on the reference path and capture the final path from approximately 16 m onward: the color should transition from green to yellow and red near the observed stop point.

## Slide-ready summary

| Item | Result |
|---|---|
| Individual module checks | Completed |
| End-to-end static-obstacle output | Stops at 27 m |
| End-to-end dynamic-obstacle output | Also stops at 27 m |
| Remaining integration issue | Dynamic classification must use `probability` in `motion_velocity_planner.py` |
