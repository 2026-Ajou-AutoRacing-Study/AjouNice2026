# SOA boundary-aware candidate search

## Implemented scope

- Map files unchanged. Existing drivable-area generation remains the hard outer corridor.
- PO's existing yaw-aware rectangle/corner test is extracted into
  `autoware_motion_utils/trajectory/drivable_area_footprint.hpp`; PO delegates to it.
- SOA uses the same corner mode (matching the current PO polygon-mode=false setting).
  Entry and return intervals are checked, with position interpolation at <=0.1 m.
  The former half-width-minus-max_deviation shortcut is no longer used in this gate.
- Up to 16 entry/return combinations, using interval-length ratios 1, .75, .5, .35.
  Entry ends and return starts stay fixed. Already-started intervals are not moved.
  This is bounded sampling, not a proof that no feasible trajectory exists.
- Reject invalid intervals, ego discontinuity, outer-corridor departures, excessive
  jerk, steering curvature or current-speed lateral acceleration, static target
  overlap, and candidates rejected by existing predictive safety checks.
- Trial safety checks restore the hysteresis count; only the final check advances
  it once per cycle.
- Lexicographic selection: current-lane/intersection corridor, same-direction
  corridor, then configured opposite-direction corridor; minimize maximum corner
  intrusion depth, estimated opposite-lane time, opposite-lane distance and
  outside-own-corridor distance, then prefer less interval shortening. Dwell time
  uses current planning speed, not a forward simulated acceleration profile.
- Relaxed lane crossing is limited to marked/virtual separators. `road_border`,
  curbs and unknown boundary types are not assumed traversable.
- Existing activation does not authorize newly rejected shift lines. Rejected
  maneuvers retain a stop target, including when comfortable stopping distance is
  insufficient. Downstream velocity smoothing and validators remain active.
- Approval policy is now `per_avoidance_maneuver` so the first search includes
  both entry and return. Their tuning parameters remain independent.

## Map audit

In `lanelet2_map_v81.osm`, the recorded local lanelets are:

| Lanelet | Left boundary | Right boundary | Both types |
|---|---|---|---|
| 200291 | 100395 | 100396 | road_border |
| 200346 | 100378 | 100379 | road_border |

These attributes cannot identify a painted centerline versus a physical road edge.
Do not relabel them without geometry/semantic confirmation. Existing intersection
area expansion is retained. Opposite-lane fallback is not permission to cross a
physical edge. No map change is required to exercise the current-corridor search.

## Verification

- Added boundary/candidate tests: 9/9 pass.
- Existing SOA utility tests: 18/18 pass; one pre-existing disabled test remains.
- Scoped builds/install passed for motion_utils, SOA, PO and autoware_launch.
- SOA and PO devel/install Build IDs match; installed config/header match sources.
- Isolated replay uses private ROS master port 11473 and recorded odometry; it is
  not a closed-loop MORAI driving test. No replay publishes to the live master.
- Bag: `planning_rviz_20260919_031426.bag`, first 21 seconds.
- Before bundling entry/return, PO still inserted a stop around y=417.9 m.
- After bundling: `planner_logs/soa_boundary_20260919_035231/result.json`:
  190 BPP and 190 PO outputs, 0 MPT primal-infeasible logs. At sampled 13--20 s
  outputs there is no stop in y=380--440 m, unlike the original y=390.69 m stop.
- Replay does contain transient invalid-candidate stop outputs around 11.1--12 s.
  The recorded vehicle continues on its old path, not the newly generated path;
  this must not be advertised as a stop-free closed-loop result.
- BPP output period in that isolated replay: median ~0.100 s, max ~0.317 s.
  Not a worst-case CPU guarantee, and only SOA was enabled in the replay BPP.
- Final scoring replay: `planner_logs/soa_boundary_20260919_035625/result.json`.
  190 BPP + 190 PO outputs, 0 MPT failures, 0 rejected-maneuver stop logs.
  After t=11 s, no BPP/PO output has a zero-speed point in map y=380--440 m.
  The first search rejects 7/16 candidates and selects tier=0, entry_ratio=.75,
  return_ratio=1.0. Entry end is unchanged, so entry starts later.
  Output period median ~0.100 s, maximum ~0.405 s near first candidate generation;
  full-stack real-time load still requires validation. Final subsequent changes
  only guard empty/malformed generated paths; the installed build and 9 tests pass.

## Next driving gate

Restart the planning stack to load the installed binaries and approval policy.
Record BPP candidate/reference/output, PO input/output, planning factors,
validation status, odometry, steering/control debug and `/rosout`.
Check new `SOA boundary candidates` / `SOA candidate rejected` / `SOA rejected
maneuver` logs. Verify body clearance, return tracking and absence of stop/restart
chatter in MORAI before claiming mission recovery. No commit or push was performed.
