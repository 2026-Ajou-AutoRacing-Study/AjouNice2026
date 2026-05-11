# ROS Topic Interface
- 예시 파일임!! 실제 우리 인터페이스가 x.

## Sensor Input

| Topic | Type | Source | Notes |
|---|---|---|---|
| `/sensing/lidar/front/points` | `sensor_msgs/PointCloud2` | HDL32 front | UDP driver |
| `/sensing/lidar/back/points` | `sensor_msgs/PointCloud2` | HDL32 rear | UDP driver |
| `/sensing/imu/data` | `sensor_msgs/Imu` | IMU | 500 Hz target |
| `/sensing/gnss/fix` | `sensor_msgs/NavSatFix` | GPS | 1 unit |
| `/sensing/camera/front/image_raw` | `sensor_msgs/Image` | Camera | FOV 55 deg |

## Localization Output

| Topic | Type | Notes |
|---|---|---|
| `/localization/odom` | `nav_msgs/Odometry` | ego pose + twist |
| `/localization/pose` | `geometry_msgs/PoseStamped` | map-frame pose |
| `/localization/twist` | `geometry_msgs/TwistStamped` | velocity |

## Planning / Control

| Topic | Type | Notes |
|---|---|---|
| `/planning/trajectory` | TBD | final trajectory |
| `/control/command` | TBD | internal control command |
| `/Ctrl_cmd` | TBD | simulator control command |