# Base2-10K D0--D2 audit

## Scope.

- Dataset ID: `morai_temporal_sequence_v2_base2_10504_final`.
- Source: `/home/luv/morai_cap/morai_temporal_sequence_v2_base2_10504_final`.
- Unit: 1 scene = 1 clip = 8 consecutive frames.
- Total: 1,313 clips, 10,504 frames.
- This audit stops at D2. Training-index construction and temporal/velocity-label
  policy belong to D3 and are intentionally not performed here.
- The packaged source is treated as read-only. Derived outputs go under
  `artifacts/datasets/morai_temporal_base2_10k_audit`.

Reproduction:

```bash
cd ~/AjouNice2026
python3 dataset/morai_od/scripts/audit_temporal_sequence_dataset.py
```

## D0. Integrity and split audit.

| Item | Result |
| --- | ---: |
| Files | 164,517 |
| Logical size | 42,278,604,707 bytes |
| Clips / frames | 1,313 / 10,504 |
| Train / val / test clips | 919 / 197 / 197 |
| Duplicate frame ID | 0 |
| Duplicate capture ID | 0 |
| Missing retained sensor artifact | 0 |
| Missing clip metadata/timing sidecar | 0 |
| Exact scenario stem shared across splits | 0 |
| Hard integrity errors | 0 |

- Every metadata, clip sidecar and split file has a SHA-256 entry in
  `metadata_checksums.csv`.
- `timing.jsonl` is a high-rate raw `ego`/`objects` state window, not an
  8-row frame table. The audit verifies both streams exist and span all eight
  capture triggers. All 1,313 clips pass.
- Actual frame interval is mean 85.84 ms, p50 81.61 ms and p95 100.67 ms.
  This is consistent with a nominal 12 Hz capture requested through
  SaveSensorData, while preserving the measured interval for temporal input.
- 89 coarse signatures overlap across splits when only family, Ego position
  rounded to 0.5 m, heading rounded to 5 degrees and actor count are used.
  This is expected because competition road sections and mission families are
  deliberately shared. The scenario stem, scene/clip/frame/capture IDs remain
  disjoint, so this is spatial-domain overlap rather than an exact clip leak.

### State timestamp caveat for D3.

- Ego state age is mean 5.98 ms and p95 10.95 ms.
- Frame-level `Object_topic` snapshot age is p50 5.93 ms but p95 656 ms.
- 962/10,504 frames have object-state age above 100 ms. This appears in dynamic
  families as well as static families and is caused by occasional callback
  gaps around SaveSensorData, not by missing sensor payloads.
- Therefore D3 must not use only the object snapshot embedded in
  `frames.jsonl` as exact sensor-time velocity GT. It must align/interpolate the
  denser raw states in `timing.jsonl`, retain label age/quality and mask samples
  that cannot satisfy the final threshold.
- This does not invalidate D0--D2 geometry or single-frame box labels. It is a
  release gate for temporal velocity-label construction.

## D1. Object and motion distribution.

Counts below are frame-object observations, not unique physical placements.
The complete per-observation table is `object_observations.csv`.

| Class | LiDAR GT observations | Visible in at least one camera | Visibility rate |
| --- | ---: | ---: | ---: |
| Vehicle | 6,365 | 6,304 | 99.0% |
| Pedestrian | 2,304 | 2,280 | 99.0% |
| Obstacle | 19,032 | 18,403 | 96.7% |

### Distance.

| Class | 0--10 m | 10--20 m | 20--30 m | 30--50 m | 50 m+ |
| --- | ---: | ---: | ---: | ---: | ---: |
| Vehicle | 10.5% | 22.5% | 18.9% | 26.0% | 22.2% |
| Pedestrian | 36.2% | 54.4% | 9.5% | 0% | 0% |
| Obstacle | 29.6% | 34.9% | 21.8% | 13.6% | 0.1% |

- Vehicle has substantial 30--50 m and 50 m+ supervision, which is useful for
  high-speed approach scenes.
- Near vehicle observations are only 10.5%. This is not an integrity failure,
  but D3 should report distance-conditioned AP and avoid allowing far boxes to
  hide near-field regressions.
- Pedestrian is intentionally concentrated inside 20 m, matching run-out and
  crosswalk interaction range.

### Ego-relative direction.

| Class | Front sector | Left lateral sector | Right lateral sector | Rear sector |
| --- | ---: | ---: | ---: | ---: |
| Vehicle | 69.7% | 7.6% | 1.4% | 21.4% |
| Pedestrian | 75.2% | 7.3% | 11.9% | 5.6% |
| Obstacle | 61.9% | 7.5% | 6.1% | 24.5% |

- These sectors are angular buckets: front is within +/-60 degrees, side is
  60--120 degrees and rear is beyond 120 degrees. Therefore `right_side 1.4%`
  does not mean only 1.4% of vehicles are on the right half-plane.
- A separate Cartesian-quadrant check gives front-left 36.5%, front-right
  37.0%, rear-left 18.3% and rear-right 8.2%. The complete right half-plane is
  45.2%, including 408 observations inside 10 m and 674 at 10--20 m.
- The small 1.4% means vehicles spend little time almost exactly abeam of Ego,
  which is reasonable for the current lane-following/overtake clips. It is not
  a Base2 weakness or a reason for targeted recapture by itself.

### Motion and assets.

- Vehicle: moving 84.5%, stationary 13.8%, slow 1.7%.
- Pedestrian: moving 22.9%, slow 37.4%, stationary 39.7%.
- Obstacle: stationary 100%.
- Vehicle has 38 named MORAI assets plus three unmatched generic labels.
  `2015_Kia_K5` accounts for 2,342/6,365 observations. This is not a random
  catalog imbalance: MORAI spawn ID 4 is fixed to that model, and the
  roundabout plus GPS-shadow families intentionally reuse spawn 4 for its
  validated route/controller behavior. Those three families account for
  2,251/2,342 K5 observations.
- Pedestrian uses only `Man1` because the temporal generator clones
  `template["pedestrianList"][0]` from the official sample for every moving and
  standing pedestrian. Location and motion were generalized, but asset choice
  was not. This is a generator design gap rather than an intentional diversity
  policy. The first temporal baseline can still diagnose velocity learning,
  but an asset-diverse human add-on is recommended before the final general
  pedestrian detector is selected.
- Obstacle contains all 30 planned assets. The least frequent asset still has
  248 observations.
- Obstacle shape observations are compact 6,448 (33.9%), long 6,808 (35.8%)
  and tall 5,776 (30.3%). The intended three-shape balance was achieved without
  deleting difficult assets.

## D2. Lanelet and route coverage.

The v60 road Lanelet centerlines are transformed back to MORAI coordinates
using the production runtime-origin transform. Each clip's first Ego pose is
projected independently to the nearest road Lanelet and to the competition
global path.

| Metric | Result |
| --- | ---: |
| Lanelet distance p50 / p95 | 0.094 / 0.158 m |
| Clips within 3 m of a road Lanelet | 1,313 / 1,313 |
| Clips within 5 m of the competition global path | 1,192 / 1,313 |
| Unique nearest road Lanelets | 38 |
| Occupied nearest-route 20 m bins | 43 |

- All clips are on valid road Lanelets. The capture controller did not send Ego
  straight through off-road space.
- The 121 clips farther than 5 m from the single competition global path are
  concentrated in `highway_cruise_overtake`,
  `highway_lane_change_overtake` and `static_highway_obstacle`. They occupy
  parallel highway lanes by design; they are not map errors.
- Train/val/test occupy 41/34/31 nearest-route bins and 34/25/24 nearest
  Lanelets. The three splits cover the same major mission regions, while exact
  scenes remain disjoint.
- The spatial plot confirms the dataset is competition-route focused rather
  than a uniform sample of the entire K-City map. This is intentional for the
  first production model and should be stated when interpreting generic map
  performance.

Generated visualizations:

- `ego_coverage_by_family.png`.
- `ego_coverage_by_split.png`.
- `route_arc_histogram.png`.
- `object_direction_distribution.png`.

RViz inspection uses the saved v60 Lanelet, static TF and top-down layout:

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
roslaunch lanelet_map_visualizer base2_10k_dataset_coverage_rviz.launch
```

Publish the audited coverage in a second terminal:

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
python3 dataset/morai_od/scripts/publish_temporal_dataset_coverage.py
```

- Topic: `/dataset/base2_10k/ego_coverage`.
- Frame: `morai_map`.
- Saved RViz layout:
  `base_ws/src/lanelet_map_visualizer/rviz/base2_10k_dataset_coverage.rviz`.
- The launch starts the v60 Lanelet map, `map -> morai_map` static TF and RViz.
- White line: competition route.
- Colored points: all 10,504 Ego samples grouped by scenario family.

## D2 decision.

- Base2-10K passes D0 integrity and D2 road-geometry coverage.
- Object, distance and obstacle-shape coverage is sufficient to proceed to D3.
- Do not recapture before D3 solely because of route coverage.
- D3 must first solve timestamp-aligned velocity-label construction from
  `timing.jsonl`. It must also preserve distance, direction, asset and family
  breakdowns so strict lateral-sector performance and pedestrian-asset limits
  remain visible.
