# Roundabout GO after stop experiment

`roundabout.common.latch_go_after_stop` enables an experimental, non-revoking GO
decision for one roundabout module instance. The package default and normal
profile are `false`; the current experimental ROS1 launch profile explicitly
sets it to `true`. Managers also default to `false` for older profiles that omit
the parameter. Parameters are read at module-manager startup, so changing the
YAML requires restarting the behavior velocity planner.

The module must first observe measured longitudinal speed with absolute value
at most **0.01 m/s** while valid roundabout geometry is available and its
collision state is STOP. A zero target speed does not count. The measured speed
must remain within this threshold continuously for the configured duration.
`roundabout.common.go_after_stop_min_stop_duration` is **0.5 s in the current
experimental ROS1 launch profile**; the package and omitted-parameter fallback
remain `1.0` seconds.
Movement before qualification resets this timer. An observation gap over 0.5 s
also restarts an unfinished hold, rather than treating missing data as proof of
continuous stopping. The first subsequent collision-free GO, including the existing STOP-to-GO hold time,
latches the decision. A scenario that starts stationary with an already-safe
GO does not arm the latch. This does not require crossing the pass judge line or setting
`permanent_go`.

If ordinary GO arrives before the physical stop hold completes, ordinary departure
is not delayed. If the vehicle remains stationary, observations can finish the
hold; if it starts moving before qualification, the latch remains unarmed.
Once the stop has qualified, its history and committed GO retain their original
behavior until an encounter reset. The stop-duration rule does not expire an old
GO after a later long wait or add a stopline-distance restriction.

Once latched, subsequent roundabout collision predictions do not produce
`RoundaboutCollisionStop`, even if comfortable stopping remains possible.
They produce a separate `GoAfterStop` decision. Collision detection continues,
and the experimental planner's `debug/roundabout/state_trace` includes:

- `go_after_stop_enabled`, `stopped_observed`, `go_after_stop_latched`
- `go_after_stop_min_stop_duration_s`, `go_after_stop_stop_duration_s`
- `go_after_stop_suppressed_stop`
- existing `raw_collision`, collision state, and per-object TTC observations

`GoAfterStop` is a commitment override, not evidence that the current situation
is collision-free. The raw collision state may therefore remain STOP while the
decision is `GoAfterStop`. This experiment intentionally removes this module's
ability to revoke GO for a newly arriving vehicle and can increase collisions.

The latch is local to the module instance and is discarded when that module
expires. It also resets on a position jump greater than 7 m, a clock rewind, or
nonfinite input. A new stop and safe GO are required after a reset. It does not
override invalid geometry, RTC behavior in other decisions, upstream zero
velocities, obstacle cruise, MPT failure stops, or control emergency stops.

Both normal and experimental roundabout implementations support the policy.
This changes no other behavior velocity modules. Unit tests cover arming,
collision hold, persistent GO despite renewed risk, disabled behavior, and
encounter reset. Closed-loop MORAI collision and completion rates must be
evaluated separately from these software checks.
