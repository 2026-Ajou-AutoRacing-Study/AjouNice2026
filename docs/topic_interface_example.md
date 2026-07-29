# ROS Topic Interface - Example
- 예시 파일임!! 실제 우리 인터페이스가 x.

## Sensor Input

| Topic | Type | Source | Notes |
|---|---|---|---|
| `/sensor/lidar1` | `sensor_msgs/PointCloud2` | HDL32 lidar1 | UDP driver |
| `/sensor/lidar2` | `sensor_msgs/PointCloud2` | HDL32 lidar2 | UDP driver |
| `/sensor/imu` | `sensor_msgs/Imu` | IMU | 500 Hz target |
| `/sensor/gps` | `sensor_msgs/NavSatFix` | GPS | 1 unit |
| `/sensor/camera` | `sensor_msgs/CompressedImage` | Camera | UDP JPEG receiver |

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



## TOCHECK - survey on morai
```bash
rostopic list

rostopic type /토픽명

# rosmsg show 메시지타입
# github repo가 있으니 생략 가능

rostopic echo /LiDAR토픽/header
rostopic echo /IMU토픽/header
rostopic echo /Camera토픽/header
# stamp:
#   시간이 제대로 찍히는가?
#   계속 증가하는가?

# frame_id:
#   어떤 좌표계 이름인가?
#   우리가 쓰려는 TF frame 이름과 맞는가?

rostopic hz /토픽명

docs/morai_topic_survey_local.md
docs/morai_topic_survey_lab.md
```


## example
# MORAI Topic Survey - Local PC

## Environment

- Date:
- MORAI version:
- ROS version:
- OS:
- Network:
- MORAI running on:
- ROS running on:

## UDP Config

| Item | Value |
|---|---|
| Sensor config | `configs/sensors/Sensor_5.json` |
| Ego network config | `config/network/ego_network_1.json` |
| Check command | `morai_status` |

## Sensor UDP Ports

| Sensor | Protocol | Destination Port | Packet received? | Check command |
|---|---|---:|---|---|
| LiDAR | UDP | 2368 | TBD | sudo tcpdump -ni any udp port 2368 |
| Camera 1-4 | UDP | 9291/9293/9295/9297 | TBD | sudo tcpdump -ni any udp portrange 9291-9297 |
| GPS | UDP | 9301 | TBD | sudo tcpdump -ni any udp port 9301 |
| IMU | UDP | 9303 | TBD | sudo tcpdump -ni any udp port 9303 |

## ROS Topics

| Purpose | Topic | Type | frame_id | Hz | Transport | Note |
|---|---|---|---|---:|---|---|
| Ego status | /Ego_topic | morai_msgs/EgoVehicleStatus | map | TBD | UDP->ROS | port 9311 |
| Control cmd | /ctrl_cmd | morai_msgs/CtrlCmd | - | - | ROS->UDP | port 9315, type 1 |
| Object GT | /Object_topic | morai_msgs/ObjectStatusList | map | TBD | UDP->ROS | port 9317 |
| Collision | /CollisionData | morai_msgs/CollisionData | map | TBD | UDP->ROS | port 9313 |
| Competition Vehicle Status | /Competition_topic | morai_msgs/EgoVehicleStatus | map | TBD | UDP->ROS | port 9319 |
| LiDAR | /sensor/lidar1 | sensor_msgs/PointCloud2 | lidar1 | TBD | UDP->ROS | VLP-16 |
| Camera | /sensor/camera/*/compressed | sensor_msgs/CompressedImage | camera_* | TBD | UDP->ROS | four JPEG streams |
| GPS | /sensor/gps | morai_msgs/GPSMessage | gps | TBD | UDP->ROS | port 9301 |
| IMU | /sensor/imu | sensor_msgs/Imu | imu | TBD | UDP->ROS | port 9303 |

## Message Structures Checked

```bash
rosmsg show ...
