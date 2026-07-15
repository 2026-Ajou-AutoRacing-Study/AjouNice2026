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

The default validation cases use a 30 m obstacle and the speed-based stop-distance
defaults:

- Input path: 1 m spacing, 8 m/s initial velocity
- Planner base/ego velocity: 5 m/s
- Static base clearance: 3 m
- Dynamic base clearance: 5 m
- Reaction time: 0.8 s
- Comfortable deceleration: 3.0 m/s²

Expected first zero-velocity points with 1 m path spacing:

| Case | Computed clearance | Expected first stop point |
|---|---:|---:|
| Static obstacle, 0 m/s | about 11.2 m | 19 m |
| Slow dynamic obstacle, 0.5 m/s | about 17.7 m | 13 m |
| Dynamic leading obstacle, 2.0 m/s | TTC about 10.0 s | no full stop |

The integrated node reads `ObjectClassification.probability` with a
`confidence` fallback, so the static and dynamic branches are active with the
current ROS messages. Dynamic objects faster than
`slow_dynamic_stop_speed_threshold_mps` only create a dynamic stop line when
ego/object TTC is below `dynamic_stop_ttc_threshold`; otherwise they are handled
by follow/TTC velocity limiting.

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
| End-to-end static-obstacle output | Stops around 19 m for a 30 m object at 5 m/s ego speed |
| End-to-end slow-dynamic output | Stops around 13 m for a 30 m object at 5 m/s ego speed |
| End-to-end faster dynamic output | No full stop for a 30 m object moving at 2 m/s when TTC is not critical |
