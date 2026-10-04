# Base4 competition-v2 MR live validation (2026-09-30)

> **CURRENT EVIDENCE / MAIN COLLECTION HOLD.** This is a small live wiring and
> capture-quality check, not a production dataset or permission to start a
> multi-hour campaign.

## Context restored

The old `live18v2` scene files predate the zero-speed control-handoff fix.
They must not be used to judge the current generator. The current `mission`
profile saves Ego at 0 km/h, rejects excessive pre-control drift, waits for
motion/minimum speed (and stable speed on cruise), and uses the current
minimal ROS Ego/Object + UDP sensor hybrid. Other profiles differ:
`high-speed-pilot` and `competition-balanced-pilot` can load Ego at target
speed and need separate safety validation.

MR rosbridge must source ROS Noetic and `base_ws/install_aw149` only; sourcing
the personal/MRP overlay changes the `morai_msgs/EgoVehicleStatus` schema and
causes the `wheel_angle` publish error.

## Live checks

1. `rv8` (initial generator): two `mission` scenes completed, each 16/16
   triggers. The two gated bags passed raw topic/marker/timing audit. Capture
   windows had LiDAR 12.57/12.97 Hz, cameras about 29.5--30.1 Hz, Ego/Object
   about 47.7/48.5 Hz; all camera--LiDAR nearest ages were <100 ms. The
   converter and temporal validator passed for the merge scene. However the
   cruise actor entered the runner gate at ~48 m and **all 16 cruise frames
   lost their Vehicle GT** under the model's +/-30 m box range. A successful
   recorder snapshot alone is therefore not a successful training scene.
2. `rv8b` (29 m start gate): 0/3 scenes completed. First control-mode service
   calls timed out for 5 s. Ego stayed parked, but NPC traffic kept moving;
   by the time Ego accelerated the merge/cruise actors were ~90--118 m ahead
   and could not be reached before the route-motion cap. A side-pass scene
   began but failed when the actor crossed the old role-specific -22 m gate.
   Failed scenes correctly produced no snapshot bags.
3. `rv8c` (allow natural exit after six actor-positive frames): the side-pass
   scene still failed at frame 6 because the actor crossed -22 m after only
   five verified frames, while remaining inside the training box range.
4. `rv8d` (29 m radial and +/-29 m longitudinal start gate): the side-pass
   scene completed 16/16 triggers. The actor naturally exited after nine
   verified frames. The bag passed raw audit (LiDAR 13.03 Hz, Ego 48.85 Hz,
   camera age max 74.5 ms). MR heading correction -1.347 deg, geometric
   BBox-center velocity, causal three sweeps and +/-30 m conversion retained
   8 Vehicle-positive frames. Info validation and temporal quality audit
   passed with no warnings/errors. The 16 trigger frames are not 16 positive
   training frames; edge timing and range filtering still apply.

Artifacts are ignored locally under `rosbags/datasets/base4_temporal/rv8*_validation/`,
`/home/luv/morai_cap/260930_base4_rv8*_validation/`, and
`artifacts/datasets/morai_base4_rv8*_validation_raw/`.

## Code guard and remaining gate

The generator now rejects campaign tags whose MORAI scenario stem exceeds
29 bytes **before** installing files. `mission` actor gates are bounded to
29 m radial; cruise uses +/-29 m longitudinal and retains a natural pass-by
after six confirmed frames. Unit tests cover these settings. These changes
prevent accepting a wholly out-of-range clip, but do **not** solve NPC phase
drift when ROS control-mode service recovery takes several seconds.

The gated recorder also now escalates INT -> TERM -> KILL with bounded waits
on shutdown. A no-scene live test confirmed it no longer leaves the rosbag
snapshot process subscribed after Ctrl-C.

**Do not start the four-hour/main collection yet.** Before release, make
control-handoff latency independent of NPC encounter timing or fail/retry a
scene early, then repeat a multi-seed merge/cruise smoke. Require both a
reasonable scene completion rate and Vehicle-positive retained frames in
both families. Recheck the actual intended collection profile separately;
`rv8*` validated only `mission`, not the target-speed-start profiles.
