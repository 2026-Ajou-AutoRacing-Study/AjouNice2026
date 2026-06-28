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
sensor extrinsics in `configs/sensors/Sensor_1.json` are interpreted in this
same rear axle center frame.

## Dimensions

The mesh is scaled to the 2023 Hyundai IONIQ 5 exterior dimensions used for
the MORAI vehicle check:

- Length: 4.635 m
- Width: 1.890 m
- Height: 1.600 m
- Wheelbase: 3.000 m
- Front overhang: 0.845 m
- Rear overhang: 0.790 m

The baked visual bbox before the URDF origin offset is:

```text
x: [-0.790, 3.845]
y: [-0.945, 0.945]
z: [0.000, 1.600]
```

With the current URDF visual offset (`visual_z_offset=-0.3705` m), RViz
displays the mesh body bbox at approximately:

```text
z: [-0.3705, 1.2295]
```

The measurement record is in `meshes/ioniq5/ioniq5_mesh_measurements.json`.

## Preview

```bash
source /opt/ros/noetic/setup.bash
source /home/jihun/AjouNice2026/base_ws/devel/setup.bash
roslaunch ioniq5 display_ioniq5.launch
```
