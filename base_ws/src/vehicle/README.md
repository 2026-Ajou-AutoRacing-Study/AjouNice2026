# ioniq5

RViz visual mesh package for the Hyundai IONIQ 5 ego vehicle.

The source asset is kept in `source_assets/Project_Cars_Hyundai_Ioniq_5.blend`.
The RViz mesh is `meshes/ioniq5/ioniq5_base_link.dae`.

## Frame Convention

- `base_link`: rear axle center
- `+X`: forward
- `+Y`: left
- `+Z`: up

The Collada mesh vertices are baked with a rear-axle ground-projection origin.
The URDF shifts the visual and collision geometry down by the wheel radius
(`0.3705` m), so the runtime `base_link` sits at the rear axle center. MORAI
sensor extrinsics in `configs/sensors/Sensor_5.json` are interpreted in this
same rear axle center frame.

The RViz visual also applies a MORAI self-return x correction of
`-0.0500` m. The first measured baseline used visible front/roof LiDAR
self-return bands: the `z=0.45..0.65` m band differed by about `-0.093` m and
the `z=1.05..1.25` m band by about `-0.107` m. A live RViz visual check still
showed the model behind the cloud, so the displayed model was moved 0.05 m
forward from that baseline. The sensor TFs were not changed because they are
loaded from the same `Sensor_5.json` that MORAI uses.

## Dimensions

The mesh is scaled to the 2023 Hyundai IONIQ 5 exterior dimensions used for
the MORAI vehicle check:

- Length: 4.635 m
- Width: 1.890 m
- Height: 1.600 m
- Wheelbase: 3.000 m
- Front overhang: 0.845 m
- Rear overhang: 0.790 m

The baked visual bbox before the URDF origin offsets is:

```text
x: [-0.790, 3.845]
y: [-1.067, 1.067]
z: [0.000, 1.600]
```

With the current URDF visual offsets (`visual_x_offset=-0.0500` m,
`visual_z_offset=-0.3705` m), RViz displays the mesh body bbox at
approximately:

```text
x: [-0.8400, 3.7950]
z: [-0.3705, 1.2295]
```

The measurement record is in `meshes/ioniq5/ioniq5_mesh_measurements.json`.

## Preview

```bash
source /opt/ros/noetic/setup.bash
source /home/jihun/AjouNice2026/base_ws/devel/setup.bash
roslaunch ioniq5 display_ioniq5.launch
```
