# Base5 MR 동기 캡처 Network — PREPARED

2026-10-05. JSON 정적 검증/설치 및 실제12-scene 캡처 연결 통과.
결과와 제한은 [MR asset smoke](261005_base5_mr_asset_review_smoke.md)를 따른다.

## 파일과 용도

- Git 원본: `configs/network/ego_network_mr_base5_savesensordata_local.json`
- MR 로드 위치: `$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Network/25.S4.MolitComp03/ego_network_mr_base5_savesensordata_local.json`
- MR 개인 PC의 ScenarioLoad + 정지 Ego + SaveSensorData/Instance annotation 검증용이다.
  대회 LAN용 또는 full-stack 주행용 Network가 아니다.
- 기존 `ego_network_mr_calibration_hybrid.json`의 검증된6-interface 내용을 그대로
  사용했다. 새 목적별 이름을 부여했으며 프로토콜/enum/주기/포트를 추정해 바꾸지 않았다.

| 연결 | 목적 |
|---|---|
| /Ego_topic | Ego pose와 정지/드리프트 확인 |
| /Object_topic | actor UID/상태 대조, 캡처 annotation과 GT 확인 |
| /ScenarioLoad | 에이전트의 소량 시나리오 로드 |
| /ctrl_cmd | 정지/제어 요청 |
| /Service_MoraiEventCmd | 제어 모드·기어 등 runner의 기존 서비스 |
| /SaveSensorData | 동기 센서 파일 캡처 요청 |

Ego UNIQUEID=0만 포함한다. Simulator UNIQUEID=-100, SyncMode 서비스, /tf,
신호 상태, Competition/기타 상태 연결은 포함하지 않는다. Ego/Object ROS 발행 설정은
기존50Hz, rosbridge는 `ws://127.0.0.1:9090`이다. JSON의 비활성 UDP/기타 config 블록은
MORAI schema 보존용이며 이 프로필이 UDP Ego를 추가 연결한다는 뜻이 아니다.

## 현재 설정 비교와 연결 실패 해석

확인 당시 MR `NetworkInfo_2023_Hyundai_Ioniq5.json`에는 Ego15개 interface와 Simulator
항목이 있었고, SaveSensorData도 이미 정의되어 있었다. 따라서 앞선 subscriber0 결과는
필드 누락의 증거가 아니라 실제 subscription 미연결의 증거다. 새 파일명만으로 이
문제가 해결됐다고 주장하지 않는다. GUI 로드/연결 후 다시 검사해야 한다.

## 터미널 실행 순서 — SaveSensorData 수집 전용

이 절은 Base5 동기 파일 캡처용이다. 전체 UDP 속도 시험이나 Base3/Base4 연속
rosbag 수집 명령과 섞지 않는다. 각 터미널은 `ajou`로 MR workspace를 준비한다.

### 터미널 1 — 필수 ROS master

```bash
ajou
rc
```

### 터미널 2 — 필수 ROS bridge

```bash
ajou
rosparam set /use_sim_time false
roslaunch rosbridge_server rosbridge_websocket.launch
```

MR에서는 `ego_network_mr_base5_savesensordata_local.json`을 로드·연결한다.
센서 설정은 `configs/sensors/Sensor_competition_local.json`을 기준으로 유지하고,
Capture Mode와 LiDAR Instance annotation 저장을 활성화한다.
Ego/Object, 제어, ScenarioLoad, SaveSensorData는 위 Network의 ROS 인터페이스를 사용한다.

### 터미널 3 — 실시간 센서 토픽도 필요할 때만 추가

```bash
ajou
roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$AJOUNICE_ROOT/configs/sensors/Sensor_competition_local.json" \
  enable_ego_network:=false \
  enable_control:=false
```

SaveSensorData는 MR이 센서 파일과 annotation을 직접 저장한다. 이 UDP receiver가
받은 ROS 센서 토픽을 저장하는 것이 아니므로, **동기 파일 캡처 자체에는 터미널 3이
필수가 아니다.** 다만 sensor topic 검사, rosbag 병행, timing sidecar가 필요한 runner
설정에는 센서 UDP 수신이 필요하다. 센서 수신 없이 실행할 때는 해당 runner의
`--no-timing-sidecar` 등 요구사항을 확인하고, annotation completeness gate는 유지한다.
Capture Mode에서는 실시간 스트림이 중단될 수 있으므로 토픽 수신을 자동 보장하지 않는다.

### 연결 후 — 한 장 검증을 통과한 다음 수집

Ego/Object의 실제 신규 메시지, ScenarioLoad/SaveSensorData 연결을 확인한 뒤
한 장을 캡처한다. Camera4/LiDAR/Instance annotation과 대상 UID가 저장된 경우에만
준비한 manifest로 확대한다. 이 문서의 연결 완료가 10K 본 수집 승인을 뜻하지 않는다.

### 혼동 금지

- `udp enable_sensors:=false enable_lidar:=false`는 **Ego/Object·제어만 UDP**로
  실행한다는 뜻이다. ROS 수신 모드로 전환하는 옵션이 아니다.
- 위 명령과 sensor-only UDP를 함께 켜면 전체 UDP를 두 프로세스 역할로 나눈 것이다.
  Base5의 ROS Ego/Object 연결과 동시에 켜지 않는다.
- 전체 UDP 보행자 속도 시험은 별도 진단이다. 그 시험이 Base5 수집 Network를
  전체 UDP로 변경했다는 뜻이 아니다.
- Base5 학습 BBox GT는 동기 LiDAR Instance annotation이다. 보조 `/Object_topic`
  velocity는 별도 평가·분석용이며 single-frame OD의 속도 학습 label로 사용하지 않는다.

## 로드 후 검증

1. 기존 rosbridge와 ROS master는 유지한다. 같은 Ego/Object를 UDP와 ROS 양쪽에서
   중복 발행하지 않는다. UDP를 유지한다면 sensor-only로 둔다.
2. MR Network GUI에서 위 새 파일을 로드하고 연결한다. Capture Mode와 LiDAR Instance
   annotation 출력도 켜 둔다. Network JSON은 센서 annotation 옵션을 설정하지 않는다.
3. Ego/Object의 신규 메시지, ScenarioLoad subscriber, SaveSensorData subscriber를
   확인한다. ROS capture publisher를 임시 생성해야 SaveSensorData topic이 보일 수 있다.
4. 연결 실패 시 중단한다. 연결 통과 후 단일 scene/capture에서 Camera4/LiDAR/Instance
   annotation/UID가 저장되는지 검증한다. 이후에만 소량 deck을 진행한다.

센서 JSON, production alias, 초기 위치/heading 및 기존 Network 파일은 변경하지 않았다.
ScenarioLoad 시 EgoNetwork sidecar가 Network를 다시 바꿀 수 있으므로 신규 캡처 시나리오
설치에는 이 파일을 `--network-template`로 사용한다. 단순 GUI 연결 검증을 위해 모든
기존 시나리오 sidecar를 덮어쓰지 않는다.
