# 오퍼레이터 네트워크와 센서 확인

처음 실행할 때는 [앞면 터미널 입력표](OPERATOR_START.md)를 먼저 본다.

2026-10-30 기준. Ubuntu가 읽는 파일과 수신 코드를 설명한다. 본선 MORAI Network는
운영측 시나리오가 자동 설정하며, 아래 ROS adapter JSON을 MORAI에 덮어쓰지 않는다.
실행 로그·오류 대응은 [터미널 안내](OPERATOR_TERMINALS.md)를 따른다.
줄 번호는 현재 저장소 기준이며 파일 수정 시 달라질 수 있다.

## 실행 명령과 파일

경로는 저장소 루트 기준이다. 각 터미널에서 먼저
`source ~/AjouNice2026/docs/common_alias.txt`를 실행한다.

| 환경 | UDP 명령 | Network JSON | Sensor JSON |
|---|---|---|---|
| 본선 192.168.0.1 → 192.168.0.10 | `udp_lan_competition` | `configs/network/ego_network_competition_lan.json` | `configs/sensors/Sensor_competition_lan.json` |
| 팀장님 무GT 시험 10.42.0.252 → 10.42.0.1 | `udp_lan_competition_teamlead` | `configs/network/ego_network_competition_lan_jihun.json` | `configs/sensors/Sensor_competition_lan_jihun.json` |
| luv 로컬 무GT 시험 | `udp_competition` | `configs/network/ego_network_competition_local.json` | `configs/sensors/Sensor_competition_local.json` |

팀장님 개발용은 Ego·Object도 받도록 다음 명령을 사용한다. 단순 `udp_lan`은 본선 IP를
읽으므로 팀장님 개발망에서 두 경로를 생략하지 않는다.

```bash
udp_lan \
  network_config:="$AJOUNICE_ROOT/configs/network/ego_network_competition_lan_jihun.json" \
  sensor_config:="$AJOUNICE_ROOT/configs/sensors/Sensor_competition_lan_jihun.json"
```

본선용은 Ego·Object OFF, 개발용은 기본 ON이다. 센서·Competition·Collision·제어는 유지한다.
본선용 수신기는 Ego Detail·Object·GT 신호·신호 강제 제어·ScenarioLoad를 열지 않는다.
JSON에 해당 항목이 남아 있다는 것과 실제 소켓을 여는 것은 다르다.
팀장님 MORAI에는 기존 팀장님 LAN Network·Sensor를 Load→Connect한다.
본선 MORAI에는 운영측 Network와 제출·승인된 Sensor를 사용한다.

프로필 선택 코드는 [run_competition_udp.sh](../../scripts/runtime/run_competition_udp.sh#L29)
29~40행이다. 실행 없이 선택 파일을 확인할 수 있다.

```bash
bash scripts/runtime/run_competition_udp.sh --profile lan --print-command
bash scripts/runtime/run_competition_udp.sh --profile teamlead --print-command
```

실행 중 실제 ROS 파라미터 확인은 별도 진단 터미널에서 한다.

```bash
rosparam get /morai_ego_udp_receiver/network_config
rosparam get /morai_sensor_udp_receiver/sensor_config
rosparam get /morai_ctrl_cmd_udp_sender/network_config
rosparam get /morai_sensor_static_tf/sensor_config
```

## Network의 주요 IP와 포트

두 LAN JSON의 아래 행 배치는 같다. `hostIP`는 MORAI 쪽,
`destinationIP`는 ROS 쪽이며 제어는 반대 방향으로 송신한다.

| 기능 | 실제 ROS 측 동작 | 본선 Network 위치 | 팀장님 Network 위치 |
|---|---|---|---|
| 제어 771 | ROS **9094** bind → MORAI **9093** 송신 | [89행부터](../../configs/network/ego_network_competition_lan.json#L89) | [89행부터](../../configs/network/ego_network_competition_lan_jihun.json#L89) |
| Collision 261 | ROS **9092** 수신 | [158행부터](../../configs/network/ego_network_competition_lan.json#L158) | [158행부터](../../configs/network/ego_network_competition_lan_jihun.json#L158) |
| Competition 295 | ROS **9099** 수신 | [442행부터](../../configs/network/ego_network_competition_lan.json#L442) | [442행부터](../../configs/network/ego_network_competition_lan_jihun.json#L442) |

각 시작 행부터 순서대로 `hostIP`, `hostPort`, `destinationIP`, `destinationPort`다.
예를 들어 제어는 89/90/91/92행이다. **771 항목 첫머리의 일반 `config`에 있는
9090이 아니라 `EgoCtrlConfig.udpConfig`의 9093/9094가 제어 설정이다.**

실제 판정프로그램의 상태 중계는 [본선 통신 계약](FINAL_NETWORK_CURRENT.md)을 따른다.
수신기는 `destinationIP:destinationPort`에 bind하며 상태 송신 IP를 동일하게 강제하지 않는다.

## Sensor의 주요 IP와 포트

파일: [본선 Sensor](../../configs/sensors/Sensor_competition_lan.json),
[팀장님 Sensor](../../configs/sensors/Sensor_competition_lan_jihun.json).
두 파일의 아래 행 배치는 같고 IP만 다르다.

| 센서 | IP·포트 시작 행 | MORAI hostPort | ROS destinationPort | ROS 출력 |
|---|---:|---:|---:|---|
| 전방 카메라 | 23 | 9290 | **9291** | `/sensor/camera/front/compressed` |
| 좌전방 카메라 | 138 | 9292 | **9293** | `/sensor/camera/front_left/compressed` |
| 우전방 카메라 | 253 | 9294 | **9295** | `/sensor/camera/front_right/compressed` |
| 후방 카메라 | 368 | 9296 | **9297** | `/sensor/camera/back/compressed` |
| LiDAR | 485 | 2369 | **2368** | `/sensor/lidar1` |
| GPS | 564 | 9300 | **9301** | `/sensor/gps` |
| IMU | 633 | 9302 | **9303** | `/sensor/imu` |

각 시작 행부터 `hostIP`, `hostPort`, `destinationIP`, `destinationPort`다.
카메라/GPS/IMU의 ROS Topic은 같은 항목의 `rosConfig.Topic`에서 읽는다.
Sensor JSON에는 동일한 IP·포트 키가 여러 번 나오므로 이름 검색 후 센서 항목을 확인한다.

**LiDAR 예외:** Sensor JSON은 MORAI 송신 설정이다. 실제 Ubuntu LiDAR 드라이버는
[UDP launch 17행](../../base_ws/src/sensor_interface/launch/morai_udp_interface.launch#L17)의
`lidar_port` 기본값 **2368**을 사용한다. JSON만 수정해서 Ubuntu 수신 포트가 바뀌지는 않는다.

## 어느 코드가 읽는가

| 역할 | 코드와 주요 행 | 동작 |
|---|---|---|
| Network 상태 수신 | [morai_ego_udp_receiver.cpp 203행부터](../../base_ws/src/sensor_interface/src/morai_ego_udp_receiver.cpp#L203) | 타입 필터 → 수신 OFF 항목 제외 → `destinationIP/Port` bind |
| 제어 송신 | [morai_ctrl_cmd_udp_sender.cpp 59행부터](../../base_ws/src/sensor_interface/src/morai_ctrl_cmd_udp_sender.cpp#L59) | 771의 `EgoCtrlConfig` → ROS destination bind → MORAI host 송신 |
| 카메라 수신 | [morai_sensor_udp_receiver.cpp 153행부터](../../base_ws/src/sensor_interface/src/morai_sensor_udp_receiver.cpp#L153) | Sensor의 destination bind, `rosConfig.Topic` 발행 |
| GPS 수신 | 같은 파일 171행부터 | Sensor의 destination bind |
| IMU 수신 | 같은 파일 186행부터 | Sensor의 destination bind |
| LiDAR 수신 | [lidar.launch](../../base_ws/src/sensor_interface/launch/lidar.launch) | 별도 Velodyne 드라이버의 `port` 인자 |
| 센서 TF | [morai_sensor_static_tf.py 75행부터](../../base_ws/src/vehicle/scripts/morai_sensor_static_tf.py#L75) | 장착 위치·회전만 읽음. UDP bind 없음 |

따라서 본선과 팀장님 모두 `sensing_lan`을 사용해도 된다. 해당 Sensor의 장착값이 같으며,
TF 생성은 파일에 적힌 IP·포트를 쓰지 않는다. BEVFusion과 신호등도 보정용 Sensor 설정을
읽을 수 있지만 센서 UDP 소켓을 여는 것은 아니다.

## 설치본과 소스가 다를 때

alias와 profile 선택 스크립트는 저장소 파일을 읽지만 `roslaunch`는 설치된 패키지를 찾는다.
소스 편집만으로 설치 launch/C++가 바뀌지는 않는다. 설정 경로는 위 `rosparam get`으로,
실제 사용 패키지는 다음으로 확인한다.

```bash
rospack find sensor_interface
rospack find bringup
```

본선 직전 즉석 수정·전체 재빌드는 하지 않는다. 불일치가 있으면 차량 정지·운영 협의 후
확정 버전의 배포 절차를 따른다. Network/Sensor 변경은 MORAI와 ROS 양쪽 대응을 확인한다.
