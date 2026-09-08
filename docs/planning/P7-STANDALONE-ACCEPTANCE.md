# P7 standalone planning acceptance

Status: **PASS 11/11 within the frozen standalone boundary. AjouNice real-input integration remains the next gate.**

This page carries the final P7.1 evidence into the integration branch. It is not a new MORAI closed-loop run. The original immutable report and receipt remain under the Autoware2ros1 harness; their SHA-256 values are recorded in `evidence/p7-standalone-reconciliation.json`.

## Why the evidence applies to this checkout

On 2026-09-08, `planning_ws/tools/audit_source.py` checked the imported planning source against the frozen r003 manifest:

- checkpoint SHA-256: `12a4c54eaf1847f1bb54fdc4a627fe2291729869e6d287ea4edb7c0c4159989b`
- manifest entries: 4,001
- checked source files: 3,997; the four omitted entries are generated Python bytecode
- changed or missing source: 0
- planning packages: 149 frozen packages plus 5 named support packages
- duplicate packages: 0
- provider source files checked: 6,589; changed or missing: 0

The live fixture hashes also match the P7.1 receipt exactly:

| Fixture | SHA-256 |
|---|---|
| `kcity_map/lanelet_map/lanelet2_map_v61.osm` | `c8ee07fb70334a13373f8cbd54fabe95980ac1e9589bbc05382337074f089dc2` |
| `kcity_map/lanelet_map/map_projector_info_v1.yaml` | `ccb5a8199afd7e64b04b6ac8637419ec4c9cd5ab16bac68a229971a7f7c7859c` |
| `base_ws/src/planning_addons/competition_route_requester/config/kcity_v61_competition_route.yaml` | `0798028dacd508a735d77585a841dc156bff6a9022480d1644b3651a3596fa30` |

Therefore, the standalone algorithm and fixture result can be carried forward to this branch. Current base integration code, external sensor data, controller behavior, or MORAI transport are not covered by that lineage proof.

## Final same-graph v61 matrix

Every case used an isolated ROS master and the same 14-process Mission Planner to full BPP to full BVP to Path Optimizer to full MVP to Velocity Smoother to Planning Validator graph. The clean-build matrix passed 11/11 without unexpected process death.

| Case | Result | Decisive evidence |
|---|---|---|
| Route A, experimental BVP | PASS | exact 47 lanelets; 20 final outputs; validator 20/20 |
| Route B, experimental BVP | PASS | exact 57 lanelets; 20 final outputs; validator 20/20 |
| Route A, normal BVP | PASS | exact 47 lanelets; 20 final outputs; validator 20/20 |
| Safe left lane change | PASS | lane sequence progressed through the expected source and target lanes |
| Occupied target lane | PASS | collision-free merge; minimum lateral separation 3.607 m exceeded the 3.25 m envelope |
| v61 merge/intersection moving conflict | PASS | original intersection logic stopped 1.3 m upstream |
| RED to GREEN | PASS | 8 red-stop outputs; velocity recovered to 16.667 m/s after green |
| Static obstacle | PASS | MVP stopped 9.914 m before the published object |
| Moving ego | PASS | ego moved 5 m; BPP 304 and MVP 9 new outputs; validator 20/20 |
| 2 s input loss and recovery | PASS | after recovery, BPP 152 and MVP 38 outputs; cached output during loss means this is not a stale-data safe-stop proof |
| Route A to B reroute | PASS | moving disjoint request rejected with original code 60001; standstill request accepted; route +1 and final trajectory +30 |

## Exact claim boundary

This proves that the frozen planning source coherently executes the required Mission to BPP to BVP to optimizer to MVP to smoother to validator graph on the actual v61 fixtures for the matrix above. It also proves Route A/B generation, selected lane-change, intersection, signal, obstacle, reroute, moving-ego, and recovery behavior inside that harness.

It does **not** prove:

- AjouNice production topic, frame, timestamp, unit, and TF correctness
- MORAI closed-loop driving or vehicle-controller handoff
- production detector/tracker accuracy
- automatic safe stop while planning inputs are stale; only post-loss recovery was proved
- Boundary Departure Prevention with real controller rollout/control/steering feedback
- all 149 packages running as 149 simultaneous nodes; 149 is the installed package boundary

The next acceptance gate is therefore the AjouNice output-only integration followed by controller closed-loop testing. The detailed input and approval sequence is in `BASE-WS-INTEGRATION-GUIDE.md`.

## Post-P7 test-only checkpoint

The P7.1 runtime acceptance remains tied to checkpoint r002 above. A later all-package `run_tests` audit found three ROS1 test sources that still assigned native `ros::Duration` or `ros::Time` directly to frozen builtin-interface payloads. Checkpoint r003 (`ef390e4751e938612c241d0eb129acd500015929555bf776d273cbde8709d28a`) changes only those test sources; the two affected packages now pass 12/12 and 16/16 tests. No runtime source or P7 fixture changed, so this delta does not silently relabel the old 11/11 run as a new runtime run. See [planning-test-audit-r001.json](evidence/planning-test-audit-r001.json).

Checkpoint r004 (`ed8f8e71a5dd91f0fc2152c0fdc5c5036bd1a8a749a707e0679ab007f8fc63b9`)
adds only BPP/BVP test-fixture compatibility. On a fresh ROS master,
`autoware_behavior_velocity_planner` passes 76 tests with zero failures or skips.
BPP direct-core fixture work remains open, and the P7.1 runtime claim is unchanged.
See [planning-test-fixture-r002.json](evidence/planning-test-fixture-r002.json).

Checkpoint r005 (`e9ab3cbc8976ef541dc48f98ce64a3a32e2495d00a5fd5a098e8a7c0873713af`)
then closes the BPP transport and Mission to BPP to BVP to MVP fixture gaps.
The isolated BPP package result improved from 14 to 2 catkin-reported failures
out of 58 tests. Both reports are the single preserved direct-core side-shift
activation oracle; its expectation was not weakened. See
[planning-test-fixture-r003.json](evidence/planning-test-fixture-r003.json).

Checkpoint r006 (`b8a0fc246224634cdbedd0cdf9bf38f4b451029039577bdd3c75f7e81d49ada3`)
closes that final BPP oracle without changing production debounce: only the
immediate-request test fixture selects the source test value of 0.0 seconds.
The isolated results are now BPP 58/58 and BVP 76/76. This is still not an
all-154-package test acceptance. See
[planning-test-fixture-r004.json](evidence/planning-test-fixture-r004.json).

Checkpoint r007 (`fbccdb518506ceec911210b4479d32207a5e50db15c11164b0a9fd6007fde116`)
loads the committed vehicle and checker parameter fixtures for the full
trajectory-checker tests. Its isolated package result is 52/52 passing,
including the long diagnostics suite. See
[planning-test-fixture-r005.json](evidence/planning-test-fixture-r005.json).

Checkpoint r008 (`5b5bcf2477d0d53a36fbf5ba006066f5f332dc5dcf6d4899dd08c7331ba9ac7b`)
closes nine Behavior Velocity scene-module package suites using their committed
ROS1 parameter files: 124 tests, zero errors, failures, or skips, with a fresh
master per package. See
[planning-test-fixture-r006.json](evidence/planning-test-fixture-r006.json).
