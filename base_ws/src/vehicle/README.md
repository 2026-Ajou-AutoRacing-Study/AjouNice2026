# ioniq5

RViz visual mesh package for the Hyundai IONIQ 5 ego vehicle.

The source asset is kept in `source_assets/Project_Cars_Hyundai_Ioniq_5.blend`.
The RViz mesh is `meshes/ioniq5/ioniq5_base_link.dae`.

## Frame Convention

- `base_link`: rear axle center on the ground
- `+X`: forward
- `+Y`: left
- `+Z`: up

The Collada mesh is baked into this ROS frame, so the URDF uses zero visual
rotation. At runtime the MORAI ego frame behaves like a rear axle wheel-center
frame, so the URDF lowers the visual/collision geometry by the tire radius plus
a small LiDAR self-return correction.

## Dimensions

The mesh is scaled to the 2023 Hyundai IONIQ 5 exterior dimensions used for
the MORAI vehicle check:

- Length: 4.635 m
- Width: 1.890 m
- Height: 1.600 m
- Wheelbase: 3.000 m
- Front overhang: 0.845 m
- Rear overhang: 0.790 m

The final visual bbox is:

```text
x: [-0.790, 3.845]
y: [-0.945, 0.945]
z: [0.000, 1.600]
```

With the current URDF visual offset (`visual_z_offset=-0.410` m), RViz displays
the mesh body bbox at approximately:

```text
z: [-0.410, 1.190]
```

The measurement record is in `meshes/ioniq5/ioniq5_mesh_measurements.json`.

## Preview

```bash
source /opt/ros/noetic/setup.bash
source /home/jihun/AjouNice2026/base_ws/devel/setup.bash
roslaunch ioniq5 display_ioniq5.launch
```
