# R028 full-lifecycle candidate boundary

This is a candidate-only dispatch from the frozen upstream package. It is not
an update to the partial `source-closure-001` candidate and does not modify the
official AW149 run.

## Included source surface

- Standard lifecycle: `src/manager.*`, `src/scene.*`, `src/debug.cpp`,
  `src/utils.*`.
- Experimental lifecycle: `src/experimental/manager.*` and
  `src/experimental/scene.*`.
- Source configuration, plugin manifest, and tests are retained as conversion
  inputs; only ROS1-rendered test executions listed below are evidence.

## Required conversion order

1. Catkin/package and generated ROS1 message headers, without changing the
   lifecycle algorithms.
2. Node, time, parameter, polling-subscriber, publisher, and callback facade.
3. Source-equivalent map geometry for start, stop, and end lines.
4. V2X state matching/freshness plus the NONE, REQUESTING, PASSING,
   FINALIZING, FINALIZED command state machine.
5. Candidate build, plugin discovery, map-backed lifecycle oracle, and BVP
   experimental-node transport gate.

The prior partial candidate is explicitly insufficient because it only renders
the stop-line REQUESTING/PASSING subset and omits start/end-line lifecycle,
finalization acknowledgement, deadlines, and command custom tags.

## Current implementation state

The experimental manager and scene facade now use typed `ros::Subscriber`,
`ros::Publisher`, `ros::Time`, ROS1 parameter reads, callback-retained V2X
state, ROS1 generated messages, and the existing ROS1 experimental
`SceneModuleInterface` NodeHandle constructor. The lifecycle branches were
not simplified during that translation.

## Executed candidate evidence

- The complete standard and experimental surface links as
  `libautoware_behavior_velocity_virtual_traffic_light_module.so` in the
  R028-only build tree.
- The ROS1-rendered upstream utility suite passes 9/9 assertions, including
  stop-point replacement versus interpolation, negative offsets, out-of-range
  failure, and end-line collision detection.
- The R028-only install tree passes the private-master plugin gate: both the
  standard and experimental pluginlib classes load and initialize; the
  experimental class registers the V2X-state subscriber and infrastructure
  command publisher.
- The map-backed lifecycle oracle passes against a one-lane fixture with
  start/stop/end regulatory lines. It rejects an otherwise-approved state
  whose `type` differs despite the same `id`, then verifies the command trace
  `NONE -> REQUESTING -> PASSING -> FINALIZING -> FINALIZED`, including stop
  insertion before the stop line and before an unfinalized end line.
- The gate installs the exact just-built candidate before every run. This
  prevents pluginlib from loading a stale installed `.so` while a newer build
  tree is being tested.

The exercised cases establish the candidate-only V2X lifecycle surface; they
do not promote the frozen official AW149 counter. The unapproved and stale
branches remain source-preserved and are covered by the same stop-line path,
but do not yet have separate fixture rows.

## 2026-09-04 foreground replay

The candidate install was refreshed from the exact R028 build before each
private-master run. The ROS1-rendered utility suite passed 9/9, the standard
and experimental pluginlib gate passed 2/2, and the map-backed lifecycle gate
passed its type-and-id rejection plus complete command-state trace. The
fixture's empty map-version warning remains a fixture limitation, not a
promotion waiver.
