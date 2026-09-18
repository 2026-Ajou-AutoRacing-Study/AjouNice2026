# LC speed transition and velocity-smoother diagnostics

## Scope

Fix the immediate-LC path-shifter execution profile; enable and record smoother filter outputs.
No lanelet IDs, route preferences, map limits, collision checks, or smoother limits are changed.
Frenet/terminal fallback speed policy is not redesigned by this patch.

## Root causes and implementation

1. `get_lane_changing_metrics` used the last preparation point's speed as an execution-wide cap.
2. Nonzero preparation duration could restrict preparation acceleration to zero; this same sample
   was then reused as the execution acceleration. Removing only the speed cap did not fix the bag.
3. Immediate-LC execution now samples independently when preparation duration is nonzero.
   Positive-acceleration candidates get a spatial speed profile using current/target road-reference
   limits, the configured global speed cap, forward acceleration and backward deceleration bounds.
   Road-segment endpoint minima prevent accelerating before a speed-limit increase.
4. LC duration and terminal speed are updated from this profile. Collision prediction integrates
   the same output speeds to the actual maneuver end. Approved paths additionally retain the
   measured-motion prediction. Existing boundary, footprint and collision rejection remain enabled.
5. `publish_debug_trajs` is enabled in the actual `scenario_common.launch`. The RViz launch topic
   list and the recorder plugin's fallback list both include all eight filter trajectories:
   raw, external limit, lateral acceleration, steering rate, time resampling, forward, backward,
   and merged filtering. Final smoother output was already recorded.

## Validation

- Utility regression suite: 16 tests, including low/high/low speed transitions, acceleration and
  braking bounds, constant limits, impossible braking, and spatial collision-prediction agreement.
- Installed launch replay, isolated ROS master 11592: all eight debug topics emitted 26 nonempty
  messages each; `publish_debug_trajs=true`. Source and installed launch match. Both source and
  installed RViz recording lists contain all eight topics.
- Latest bag: `planning_rviz_20260918_213344.bag`.
- First patch replay still selected zero acceleration: caught the second root cause above.
- Corrected open-loop replay: initial LC selected +3.7 m/s²; at the recorded 99 s pose BPP target
  increased to 71.36 km/h instead of 59.25. Later open-loop poses diverge from the new geometry:
  101 s onward included a zero-speed path. This replay is NOT a driving-success test.
- Separate idealized BPP-following model replay (not vehicle dynamics, not downstream smoother):
  approximately 59.5 km/h at 98 s, 69.7 at 99 s, 83.0 at 100 s, 109.6 at 102 s;
  reached path lanelet 201469 near 110 s and 201439 near 112 s; no ROS ERROR logs.
  This supports removal of the BPP cap, not MORAI closed-loop success or lateral-dynamics validation.
- Replay verified the installed LC shared library was loaded.

Artifacts (ignored diagnostic directory):

- `planner_logs/lc_overlap_spatial_speed2_213344/result.json`
- `planner_logs/lc_overlap_spatial_speed_model_213344/result.json`
- `planner_logs/smoother_debug_check/result.json`
- `/tmp/lc-spatial-final-tests.log`

## Activation / remaining verification

LC library, tier4 planning launch, and bringup recorder plugin/launch are built and installed.
The live stack was NOT restarted. Restart planning and RViz before the next recording; an existing
RViz panel retains its old topic list. No perception restart is required by these edits.
Use a fresh bag to verify LC tracking, lateral limits, object interactions and smoother braking.
The previously observed 134→84 km/h smoother deceleration was instrumented, not disabled or fixed.
