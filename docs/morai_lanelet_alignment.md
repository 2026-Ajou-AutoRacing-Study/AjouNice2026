# MORAI k-city to Lanelet Map Alignment

This alignment fixes the relationship between the Autoware/Lanelet `map`
frame and the MORAI `morai_map` frame. It is a static map-to-map transform,
not a runtime GPS conversion.

## Frame Decision

- `map`: Lanelet2 LocalCartesianUTM frame from `kcity_map/map_projector_info_v1.yaml`
- `morai_map`: MORAI k-city local frame from the current live simulator account
- Runtime static TF: `map -> morai_map`

## Source Coordinates

Current MORAI competition runtime GPS offsets:

```text
/sensor/gps:
  eastOffset: 302595.0
  northOffset: 4124145.0

global coordinate system:
  +proj=utm +zone=52 +ellps=WGS84 +units=m +no_defs
```

The previous extracted `k-city.scene` origin was
`[302459.942, 4122635.537, 28.991]`, but the live competition account uses the
runtime GPS offsets above. Use the runtime offsets for simulator alignment.
`/Ego_topic.position.z` is already in an altitude-like frame, so the static
`map -> morai_map` z translation is `0.0`.

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
  x: 131.534421814
  y: 545.922194568
  z: 0.0
rotation:
  roll:  0.0
  pitch: 0.0
  yaw:   0.000326632868 rad
         0.018714685 deg
```

The unrefined translation from runtime GPS origin to Lanelet map origin is:

```text
morai_origin_utm - lanelet_origin_utm
  x: 131.559130220
  y: 545.911338186
  z: 0.0
```

## Runtime Validation

The current yaw refinement was calculated from
`rosbags/raw/raw_sensor_0629_1718.bag`, which uses the same runtime GPS offsets
as the static TF:

```text
matched samples:
  1188 /sensor/gps messages matched to interpolated /Ego_topic.position

fit:
  map_xy = R(yaw) * morai_xy + translation
  yaw: 0.000326632868 rad
       0.018714685 deg
  translation: [131.534421814, 545.922194568, 0.0]

residuals:
  before rms: 0.103772 m
  after rms:  0.100785 m
  after p50:  0.045755 m
  after p95:  0.219235 m
```

The fitted translation differs from the raw GPS-origin delta by
`[-0.024708406, 0.010856382, 0.0]` m. That centimeter-scale shift keeps the
small yaw correction centered on the live trajectory instead of rotating around
the raw origin.

## Previous Scene Feature Check

The older extracted `k-city.scene` origin was also checked against the Lanelet
map geometry:

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

Small ICP refinements on that older scene improved mean nearest distance by only
millimeters while producing layer-dependent corrections:

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

Those values are kept as historical geometry evidence only; the live simulator
alignment should follow the current runtime GPS offsets above.

## Runtime Use

The sensor interface launch publishes the static transform through:

```bash
roslaunch sensor_interface map_static_tf.launch
```

Run exactly one dynamic map TF source:

```bash
# Rosbag replay before localization is ready.
roslaunch bringup maptf_morai_bag.launch

# Live MORAI simulator before localization is ready.
roslaunch bringup maptf_morai.launch

# After localization is ready and publishing /localization/odom.
roslaunch bringup map_tf.launch
```

The two MORAI Ego launches publish `morai_map -> base_link`, so the runtime
tree becomes:

```text
map -> morai_map -> base_link
```

`map_tf.launch` is different: it trusts `/localization/odom` frame ids by
default, with fallback `map -> base_link` if the message frame ids are empty.
That is the post-localization path, not a MORAI Ego fallback path.

MORAI also publishes a raw `/map -> /base_link` TF. That raw `map` is MORAI's
local MGeo frame, not the Lanelet/VMB `map` frame. Keep that raw TF isolated
unless deliberately debugging MORAI bridge TF rewriting.

For RViz inspection, publish the Lanelet OSM as `visualization_msgs/MarkerArray`:

```bash
roslaunch lanelet_map_visualizer kcity_lanelet_map.launch
```

or enable it with the sensing stack:

```bash
roslaunch bringup sensing.launch enable_lanelet_map_visualization:=true
```

The marker node publishes both `/map/vector_map_marker` and the Autoware.AI-style
legacy topic `/lanelet2_map_viz`, using the same `LocalCartesianUTM` projector
metadata that defines the `map` frame. Lanelet fill polygons are disabled by
default because the line layers are cleaner for alignment inspection; pass
`publish_lanelet_fill:=true` only when filled areas are explicitly needed.
