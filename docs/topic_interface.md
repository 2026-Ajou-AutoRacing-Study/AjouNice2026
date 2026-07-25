# MORAI UDP topic interface

MORAI와 이 저장소 사이의 외부 통신은 UDP만 사용한다. `sensor_interface`의 C++ 노드가 UDP 패킷을 기존 ROS 메시지로 변환하므로, 제어기와 인지/위치추정 모듈은 ROS 토픽을 그대로 사용한다.

## 설정 파일

- 센서: `configs/sensors/Sensor_4.json`
- Ego Network: `configs/network/ego_network_1.json`
- UDP 수신: `roslaunch sensor_interface morai_udp_interface.launch`
- 센서 TF/RViz/차량 모델: `roslaunch bringup sensing.launch`

별도 `morai_udp.yaml`은 없다. C++ 노드가 두 MORAI JSON을 직접 읽는다.

## MORAI -> ROS

| UDP destination | ROS topic | ROS type |
|---:|---|---|
| 9311 | `/Ego_topic` | `morai_msgs/EgoVehicleStatus` |
| 9313 | `/CollisionData` | `morai_msgs/CollisionData` |
| 9317 | `/Object_topic` | `morai_msgs/ObjectStatusList` |
| 9319 | `/Competition_topic` | `morai_msgs/EgoVehicleStatus` |
| 9301 | `/sensor/gps` | `morai_msgs/GPSMessage` |
| 9303 | `/sensor/imu` | `sensor_msgs/Imu` |
| 9291 | `/sensor/camera/front/compressed` | `sensor_msgs/CompressedImage` |
| 9293 | `/sensor/camera/front_left/compressed` | `sensor_msgs/CompressedImage` |
| 9295 | `/sensor/camera/front_right/compressed` | `sensor_msgs/CompressedImage` |
| 9297 | `/sensor/camera/back/compressed` | `sensor_msgs/CompressedImage` |
| 2368 | `/sensor/lidar1` | `sensor_msgs/PointCloud2` |

카메라 토픽이 `/compressed`로 끝나므로 `rqt_image_view`에서 transport 이름을 정상 인식한다.

## ROS -> MORAI

| ROS topic | ROS type | UDP target | Note |
|---|---|---:|---|
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` | 9315 | `longlCmdType=1`, accel/brake 방식 |

제어기는 이전처럼 `/ctrl_cmd`에 타이어 조향각 `[rad]`을 publish한다.
`morai_ctrl_cmd_udp_sender`는 차량의 최대 조향각을 이용해
`steering_rad / max_steer_rad`로 `[-1, 1]` 정규화한 뒤 MORAI
`#MoraiCtrlCmd$` 55-byte UDP 패킷으로 변환한다.

Scenario Load는 `/morai/scenario_load_udp`
(`morai_msgs/MoraiScenarioLoadSrv`) 서비스를 호출한다.
`morai_scenario_load_udp_sender`가 요청을 `#ScenarioLoad$` 69-byte UDP
패킷으로 변환하며, `ego_network_1.json`의 `PUBSUB_TYPE=783` 주소를 사용한다.
`file_name`에는 MORAI에 저장된 시나리오 이름을 전달한다. `.json` 확장자는
있어도 자동으로 제거된다.

```bash
rosservice call /morai/scenario_load_udp "request:
  file_name: 'loadstart.json'
  load_network_connection_data: true
  delete_all: false
  load_ego_vehicle_data: true
  load_surrounding_vehicle_data: true
  load_pedestrian_data: true
  load_obstacle_data: true
  set_pause: false"
```

서비스의 `result=true`는 UDP 패킷 전송 성공을 뜻한다. UDP에는 응답 패킷이
없으므로 실제 로드 성공 여부는 `/Ego_topic` 재수신과 초기 위치로 확인한다.

## TF ownership

- `base_link -> sensor`: `Sensor_4.json`을 읽는 `morai_sensor_static_tf.py`
- 동적 `map -> base_link`: 선택한 localization 모듈
- MORAI `/tf`: 사용하지 않음

## 실행

```bash
source /opt/ros/noetic/setup.bash
source ~/AjouNice2026/base_ws/devel/setup.bash
roslaunch sensor_interface morai_udp_interface.launch

# 별도 터미널
source /opt/ros/noetic/setup.bash
source ~/AjouNice2026/base_ws/devel/setup.bash
roslaunch bringup sensing.launch start_rviz:=false enable_vehicle_model:=false
```

MORAI GUI에서는 Sensor 설정에 `Sensor_4.json`, Ego-0 Network 설정에 `ego_network_1.json`을 각각 로드해야 한다. 파일을 수정한 것만으로 이미 실행 중인 MORAI에 자동 반영되지는 않는다.
