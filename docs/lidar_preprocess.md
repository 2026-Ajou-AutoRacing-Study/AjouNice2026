# LiDAR Preprocessor Design

## Purpose

This document specifies the perception-side LiDAR preprocessor for the
AjouNice2026 ROS1 stack. It is intentionally separate from FAST-LIO. FAST-LIO
continues to consume its own raw LiDAR input and perform its own internal
undistortion. The preprocessor described here produces a high quality
multi-LiDAR point cloud for perception, visualization, and later BEV-style
pipelines.

The current naive concat path transforms each LiDAR cloud to `base_link` and
appends the points. That is enough for a loose overlay, but it does not correct
motion distortion inside a rotating LiDAR scan. At about 50 kph, a vehicle moves
roughly 0.97 m during a 70 ms scan, so points acquired at different azimuths can
appear shifted when all points are visualized as if they were captured at one
instant.

## Target Pipeline

```text
/sensor/lidar1 raw + ego motion -> /sensor/lidar1/deskewed
/sensor/lidar2 raw + ego motion -> /sensor/lidar2/deskewed

/sensor/lidar1/deskewed
/sensor/lidar2/deskewed
  -> time-synchronized field-preserving concat
  -> /sensor/lidar_concat/deskewed
```

The existing `/sensor/lidar_concat` topic remains useful as a comparison
baseline. The new preprocessor publishes a separate corrected topic.

## Relation To Autoware

The ROS1 implementation follows the same split used by Autoware Universe's
pointcloud preprocessor:

- `distortion_corrector`: corrects distortion inside a single LiDAR scan using
  ego motion. Autoware uses twist and can use IMU angular velocity.
- `concatenate_and_time_synchronize_node`: synchronizes multiple LiDAR clouds,
  chooses a reference timestamp, and can compensate cloud-level motion before
  concatenation.

This repository does not run those ROS2 components directly. Instead, the same
behavior is implemented in ROS1 C++ under `sensor_interface`.

## Terminology

- Scan: one `sensor_msgs/PointCloud2` message from one LiDAR.
- Point time: the exact acquisition offset of one point, read from the `time`
  field in the cloud. In the MORAI/Velodyne path this is a floating point offset
  relative to the cloud header stamp.
- Deskew / distortion correction: point-wise correction inside one LiDAR scan.
- Concat motion compensation: cloud-wise correction between LiDAR messages with
  different header timestamps.

## Confirmed Live Inputs

The current MORAI/Velodyne topics expose the fields required for deskew:

```text
/sensor/lidar1 fields:
  x float32
  y float32
  z float32
  intensity float32
  ring uint16
  time float32

/sensor/lidar2 fields:
  x float32
  y float32
  z float32
  intensity float32
  ring uint16
  time float32
```

The current IMU topic provides angular velocity:

```text
/sensor/imu:
  sensor_msgs/Imu
  frame_id: imu
  angular_velocity.z: available
```

The static TF currently has no rotation between `base_link` and `imu`, so
`angular_velocity.z` can be used directly after a TF-based vector transform.

## Motion Modes

The preprocessor has two user-facing modes. Both modes are converted to the same
internal ego-motion sample so that the deskew math does not care where the data
came from.

```cpp
struct EgoMotionSample {
  ros::Time stamp;
  double x;    // world frame
  double y;    // world frame
  double yaw;  // world frame
  double vx;   // base_link frame
  double vy;   // base_link frame
  double wz;   // base_link yaw rate, from IMU
};
```

### `morai` Mode

Inputs:

```text
/Ego_topic:
  position.x/y -> x/y
  heading      -> yaw
  velocity     -> vx/vy after configurable frame conversion

/sensor/imu:
  angular_velocity.z -> wz
```

This mode is for simulator development and quick validation before the real
localization stack is ready. It intentionally uses IMU for `wz` so that angular
velocity handling is the same as `odom` mode.

### `odom` Mode

Inputs:

```text
/localization/odom:
  pose         -> x/y/yaw
  twist.linear -> vx/vy after configurable frame conversion

/sensor/imu:
  angular_velocity.z -> wz
```

This mode is the competition-style interface. The localization stack may produce
`/localization/odom`, while the IMU supplies yaw rate.

## Important Frame Convention

Internal velocities are normalized to `base_link`:

```text
vx: forward velocity in base_link
vy: left velocity in base_link
wz: yaw rate around base_link z
```

MORAI `/Ego_topic.velocity` must be checked while the vehicle is moving. The
sample available during implementation was near zero speed, so it was not enough
to prove whether the velocity vector is already in the vehicle frame or in the
MORAI world frame. The YAML keeps this explicit:

```yaml
morai_velocity_frame: "base_link"   # base_link | world
odom_twist_frame: "base_link"       # base_link | world
```

If a live moving check shows MORAI velocity is world-frame, set
`morai_velocity_frame: "world"`.

## Distortion Corrector Node

Executable:

```text
lidar_distortion_corrector
```

One instance is launched per LiDAR.

Inputs:

```text
cloud_topic: /sensor/lidar1 or /sensor/lidar2
imu_topic: /sensor/imu
ego_topic: /Ego_topic                 # morai mode
odom_topic: /localization/odom        # odom mode
TF: base_link -> lidar frame
```

Output:

```text
/sensor/lidar1/deskewed
/sensor/lidar2/deskewed
```

Algorithm:

1. Read `x`, `y`, `z`, and `time` fields directly from `PointCloud2`.
2. Transform each point from the LiDAR frame to `base_link`.
3. Compute point acquisition offset:

   ```text
   dt = point.time
   point_stamp = cloud.header.stamp + dt
   ref_stamp = cloud.header.stamp
   ```

4. Use ego motion at `ref_stamp`:

   ```text
   vx, vy from MORAI or odom
   wz from IMU
   ```

5. Apply 2D constant-twist correction from `point_stamp` to `ref_stamp`.
6. Write corrected `x`, `y`, `z` back into the copied `PointCloud2`.
7. Preserve all original fields, including `intensity`, `ring`, and `time`.
8. Publish in `base_link`.

The first implementation uses 2D correction because the observed high-speed
shift is dominated by forward velocity and yaw rate. The configuration leaves a
`correction_mode` field so a later 3D mode can be added without changing the
topic contract.

## Advanced Concat Node

Executable:

```text
lidar_advanced_concat_filter
```

Inputs:

```text
/sensor/lidar1/deskewed
/sensor/lidar2/deskewed
imu_topic: /sensor/imu
ego_topic or odom_topic, depending on mode
```

Output:

```text
/sensor/lidar_concat/deskewed
```

Algorithm:

1. Approximate-time synchronize two deskewed clouds.
2. Choose the earliest input header timestamp as the output timestamp.
3. Transform each cloud to the configured output frame.
4. If enabled, compensate each whole cloud from its header timestamp to the
   earliest timestamp using the same 2D ego-motion model.
5. Concatenate without converting to `pcl::PointXYZI`.
6. Preserve the full point schema, including `ring` and `time`.

The cloud-level compensation is separate from point-wise deskew. It only handles
the timestamp difference between LiDAR messages.

## YAML Contract

Recommended config file:

```yaml
motion_mode: "morai"          # morai | odom
base_frame_id: "base_link"
output_frame_id: "base_link"

imu_topic: "/sensor/imu"
ego_topic: "/Ego_topic"
odom_topic: "/localization/odom"

morai_velocity_frame: "base_link"
odom_twist_frame: "base_link"

correction_mode: "2d"
motion_timeout_sec: 0.2
imu_timeout_sec: 0.2
buffer_duration_sec: 2.0

fields:
  x: "x"
  y: "y"
  z: "z"
  time: "time"
```

## Verification Plan

1. Confirm LiDAR fields:

   ```bash
   rostopic echo -n 1 /sensor/lidar1/fields
   rostopic echo -n 1 /sensor/lidar2/fields
   ```

2. Confirm IMU yaw rate:

   ```bash
   rostopic echo -n 1 /sensor/imu/angular_velocity
   ```

3. Confirm static frames:

   ```bash
   rosrun tf tf_echo base_link lidar1
   rosrun tf tf_echo base_link lidar2
   rosrun tf tf_echo base_link imu
   ```

4. Launch corrected preprocessor and compare:

   ```bash
   rostopic echo -n 1 /sensor/lidar_concat/deskewed/fields
   rostopic hz /sensor/lidar_concat /sensor/lidar_concat/deskewed
   ```

5. In RViz at about 50 kph, overlay:

   ```text
   /sensor/lidar_concat
   /sensor/lidar_concat/deskewed
   robot_description / URDF
   ```

6. Check whether the roof/self-return and static road geometry are less shifted
   in the deskewed cloud.

## Remaining Open Checks

- Determine whether `/Ego_topic.velocity` is `base_link` frame or world frame
  while the vehicle is actually moving.
- Confirm IMU `angular_velocity.z` sign against heading change during a left or
  right turn.
- Decide later whether downstream perception should preserve point `time`,
  zero it, or rewrite it relative to the concat output timestamp. The initial
  implementation preserves it to avoid losing raw timing information.
