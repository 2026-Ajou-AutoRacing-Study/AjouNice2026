# K-City v66 map validation report

## Lane-change representation

Lane changes are represented by existing Lanelet2 lateral adjacency and the
route segment's `preferred_primitive`.  Parallel lanes belong to the same
route segment; an artificial diagonal lanelet is not added.  The behavior path
planner uses the routing graph and passable shared boundary to generate the
lateral path.

For the competition highway route, the DefaultPlanner segment around
`200101` contains `[200142, 200152, 201434, 200101]`.  The requester changes
that segment's preferred primitive to `201434`, then applies this preferred
sequence:

```text
201434 -> 201435 -> 200137 -> 200187 -> 201443 -> 200130 -> 201441 -> 200360
```

`200137` is parallel to `200128`, rather than a missing serial lanelet.
`201443` is the middle lane at its cross-section; `200188` is the top lane.

## v65 issue causes and v66 repairs

| Reported category | v65 | v66 | Repair |
| --- | ---: | ---: | --- |
| Stop line does not intersect lanelet | 15 lines / 34 relations | 0 | Removed 39 traffic-light regulatory references added to non-intersecting internal lanelets; stop-line geometry was preserved |
| Adjacent points at the same position | 2 pairs | 0 | Removed the duplicate steep point in `200309`; regenerated the invalid `200668` centerline |
| Centerline not connected to next | 1,133 lanelets | 0 | Reused one common centerline endpoint node for each shared boundary cross-section |
| Speed above 100 km/h threshold | 38 lanelets | 0 | Capped the former 150 km/h lanelets at 100 km/h |
| Centerline points outside lanelet | 22 lanelets | 0 | Regenerated each centerline by matched left/right-boundary interpolation |
| Steep-angle point | 1 point | 0 | Removed point `161266` from the `200309` centerline |
| Missing `intersection_area` ID | 54 lanelets | 0 | Marked all seven closed intersection ways with `area=yes` so they load into the polygon layer |
| Traffic-light points in wrong order | 4 lines | 0 | Reversed `103861`, `103905`, `104150`, and `103828` |
| Intersection lanelet without RightOfWay | 23 lanelets | 0 | Removed the invalid traffic-light attachment that incorrectly made those left/right internal lanes signal-controlled |

The v65 stop lines, road boundaries, intersection polygons, roundabout
regulatory element, and competition route lanelet IDs remain in place.

## Reproduction

```bash
python3 kcity_map/scripts/kcity_map_workflow/build_v66_validation_fixes.py
python3 kcity_map/scripts/kcity_map_workflow/validate_v66_map.py
```

Expected result:

```text
attachment_errors=0
attachment_warnings=0
lanelets=1303
explicit_centerlines=1214
traffic_light_stopline_checks=68
intersection_areas=7
roundabout_members=14
sha256=aa6650f1b1c054b51c0a4dee6732e98fb5eea7bf63ff7e056acb15301ab7ce3a
```
