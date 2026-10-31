# K-City v67 intersection speed report

## Result

K-City v67 defines the seven ordinary intersections traversed by competition
Routes A/B as physical `intersection_area` polygons and sets every vehicle road
lanelet contained by each polygon to `speed_limit=30` km/h.

- Source: `lanelet2_map_v66.osm`
- Output: `lanelet2_map_v67.osm`
- SHA-256: `86736354e4bc0ba52eee72a847ad08f0dd61555efa8a905bd5853f040a02bb9a`
- Geometry changes: 0
- Intersection polygons: 7
- Vehicle lanelets at 30 km/h: 60
- Pedestrian-only crosswalks excluded from vehicle speed tags: 20

The v66 polygons already followed the road boundaries and fully enclosed their
vehicle movements, so duplicating or expanding them would have made the
collision area less precise. v67 preserves those polygon geometries. The audit
did find three road movements inside polygon `104158` that were missing the
semantic association. v67 adds them:

| Lanelet | Added direction | Intersection area | Speed |
|---|---|---|---|
| `200289` | `straight` | `104158` | 30 km/h |
| `200328` | `right` | `104158` | 30 km/h |
| `200416` | `left` | `104158` | 30 km/h |

## Competition intersections

| Order | Route passage | Polygon | Vehicle lanelets at 30 km/h |
|---:|---|---|---|
| 1 | `200178 -> 200337 -> 200287` | `104013` | `200290, 200337, 200391, 200394, 200417, 200446` |
| 2 | `200287 -> 200348 -> 200346` | `104014` | `200345, 200348, 200385, 200964, 201072, 201073` |
| 3 | `200346 -> 200291 -> 200292` | `104158` | `200289, 200291, 200327, 200328, 200416, 200426` |
| 4 | `200841 -> 200950 -> 200310` | `104016` | `200309, 200344, 200383, 200433, 200515, 200516, 200950, 200983` |
| 5 | `200258 -> 200261 -> 200315` | `104017` | `200169, 200261, 200318, 200409, 200429, 200439, 200441, 200442` |
| 6 | `200315 -> 200445 -> 200311` | `104018` | `200268, 200270, 200272, 200275, 200278, 200312, 200313, 200386, 200398, 200432, 200445, 200468, 201149, 201174` |
| 7 | `200889 -> 200220 -> 200168` | `104012` | `200160, 200171, 200177, 200220, 200392, 200393, 200413, 200414, 200434, 200443, 200444, 201070` |

The roundabout passage `200968 -> 201095 -> 201098 -> 201096 -> 201087 ->
201086` remains governed by its separate `Roundabout` regulatory element and
is outside these seven ordinary intersection speed zones.

## Definition and validation

A vehicle lanelet belongs to an intersection zone when at least 95% of its road
polygon is contained by the corresponding physical `intersection_area`. Every
such lanelet must have the same `intersection_area`, a valid
`turn_direction`, and `speed_limit=30`. Crosswalk lanelets are checked for
classification but retain pedestrian-only semantics without a vehicle speed
tag.

```bash
python3 kcity_map/scripts/kcity_map_workflow/build_v67_intersection_speed_limits.py
python3 kcity_map/scripts/kcity_map_workflow/validate_v67_intersection_speed_limits.py
python3 kcity_map/scripts/kcity_map_workflow/visualize_v67_competition_intersections.py
```

Visual evidence:

- [Competition route overview](assets/kcity_v67_competition_intersections.svg)
- [All seven polygons at local scale](assets/kcity_v67_competition_intersections_detail.svg)
