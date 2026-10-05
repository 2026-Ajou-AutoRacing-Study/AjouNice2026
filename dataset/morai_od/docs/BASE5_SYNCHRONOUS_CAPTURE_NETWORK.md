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
