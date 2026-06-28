# MORAI k-city to Lanelet Map Alignment

This alignment fixes the relationship between the Autoware/Lanelet `map`
frame and the MORAI `morai_map` frame. It is a static map-to-map transform,
not a runtime GPS conversion.

## Frame Decision

- `map`: Lanelet2 LocalCartesianUTM frame from `maps/map_projector_info_v1.yaml`
- `morai_map`: MORAI k-city MGeo local frame from `k-city.scene`
- Runtime static TF: `map -> morai_map`

## Source Coordinates

MORAI k-city MGeo `global_info.json`:

```text
global_coordinate_system: +proj=utm +zone=52 +ellps=WGS84 +units=m +no_defs
local_origin_in_global: [302459.942, 4122635.537, 28.991]
workspace_origin: [0, 0, 0]
```

Lanelet projector:

```text
projector_type: LocalCartesianUTM
map_origin:
  latitude: 37.2380038082
  longitude: 126.7730819386
  altitude: 0.0
```

Projected with UTM zone 52N:

```text
lanelet map origin UTM52:
  easting:  302463.440869780
  northing: 4123599.088661814
  altitude: 0.0
```

## Final Transform

```text
map -> morai_map
translation:
  x: -3.498869780
  y: -963.551661814
  z: 28.991
rotation:
  roll:  0.0
  pitch: 0.0
  yaw:   0.0
```

The translation is:

```text
morai_origin_utm - lanelet_origin_utm
```

## Feature Validation

After applying the origin-delta transform, the MORAI k-city geometry already
overlaps the Lanelet map tightly:

```text
MGeo link_set -> Lanelet relation centerlines
  mean nearest distance: 0.420 m
  p50: 0.344 m
  p95: 1.103 m

MGeo lane_boundary_set -> Lanelet relation boundaries
  mean nearest distance: 0.620 m
  p50: 0.523 m
  p95: 1.389 m
```

Small ICP refinements were tested but not adopted. They improved mean nearest
distance by only millimeters while producing layer-dependent corrections:

```text
links vs centerlines:
  yaw: -0.001362 deg
  final x/y: [-3.484923, -963.611587]

boundaries vs relation boundaries:
  yaw: +0.003374 deg
  final x/y: [-3.355962, -963.558077]

link+boundary vs center+boundary:
  yaw: -0.003569 deg
  final x/y: [-3.524290, -963.635817]
```

Because the ICP candidates disagree by centimeters and the source maps are both
georeferenced in UTM52, the authoritative UTM origin delta is the final static
alignment.

## Runtime Use

The bringup launch publishes the static transform through:

```bash
roslaunch bringup morai_map_static_tf.launch
```

`morai_lidar_udp.launch` includes it by default through `publish_morai_map_tf`.

