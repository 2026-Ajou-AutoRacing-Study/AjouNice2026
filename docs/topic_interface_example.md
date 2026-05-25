# ROS Topic Interface - Example
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

## ROS Bridge

| Item | Value |
|---|---|
| Bridge IP | TBD |
| Bridge Port | 9090 |
| Check command | ss -antp \| grep 9090 |
| Connected? | yes/no |

## Sensor UDP Ports

| Sensor | Protocol | Destination Port | Packet received? | Check command |
|---|---|---:|---|---|
| LiDAR | UDP | 2368 | TBD | sudo tcpdump -ni any udp port 2368 |
| Camera | UDP | 2370 | TBD | sudo tcpdump -ni any udp port 2370 |
| GPS | UDP | 2372 | TBD | sudo tcpdump -ni any udp port 2372 |
| IMU | UDP | 2374 | TBD | sudo tcpdump -ni any udp port 2374 |

## ROS Topics

| Purpose | Topic | Type | frame_id | Hz | Transport | Note |
|---|---|---|---|---:|---|---|
| Ego status | TBD | TBD | - | TBD | rosbridge |  |
| Control cmd | /ctrl_cmd | morai_msgs/CtrlCmd | - | - | rosbridge | publish target |
| Object GT | TBD | TBD | - | TBD | rosbridge |  |
| LiDAR | TBD | TBD | TBD | TBD | UDP→ROS? |  |
| Camera | TBD | TBD | TBD | TBD | UDP→ROS? |  |
| GPS | TBD | TBD | TBD | TBD | UDP or rosbridge |  |
| IMU | TBD | TBD | TBD | TBD | UDP or rosbridge |  |

## Message Structures Checked

```bash
rosmsg show ...