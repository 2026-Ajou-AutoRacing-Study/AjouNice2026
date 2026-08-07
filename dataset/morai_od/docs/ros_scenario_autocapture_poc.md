# MORAI ROS 시나리오 자동 로드·캡처 PoC

## 목적

대량 manifest와 정교한 랜덤 배치 규칙을 만들기 전에 다음 최소 파이프라인이
실제 개인용 MORAI에서 끝까지 동작하는지 검증한다.

```text
짧은 Internal Scenario JSON 3개 생성
→ MORAI 현재 맵의 Scenario 폴더에 설치
→ 시나리오별 ROS EgoNetwork sidecar 설치
→ ROS /ScenarioLoad
→ rosbridge 일시 단절 및 sidecar 기반 자동 복구
→ Ego 위치와 /Object_topic 객체 수 검증
→ ROS /SaveSensorData
→ 실제 파일 생성 확인
→ 저속으로 3 m 이동하며 6회 반복 캡처
```

자동 캡처 자체가 되지 않으면 대량 시나리오 생성은 의미가 없으므로, 먼저
정지 상태의 단일 프레임으로 로드와 저장만 검증한다. 그다음에만 Ego 이동을
켜고 마지막에 3개 시나리오 배치를 실행한다.

## 구현 파일

| 파일 | 역할 |
|---|---|
| `scripts/generate_morai_od_prototype.py` | 공식 waypoint와 MGeo로 시나리오 한 개 생성 |
| `scripts/generate_ros_capture_poc_set.py` | `od_poc_01`~`03`, manifest, ROS network sidecar 생성·설치 |
| `scripts/ros_scenario_autocapture_poc.py` | ROS 로드, network 재연결, 장면 검증, 이동, 캡처, 파일 검증 |
| `manifests/ros_capture_poc.json` | 3개 장면과 기대 객체 수, 캡처 정책 |

시나리오 stem은 MORAI legacy transport의 30-byte 제한을 피하도록
`od_poc_01`처럼 짧게 유지한다.

## Scenario와 ROS network sidecar

MORAI에서 시나리오를 로드하면 Ego가 재생성되면서 기존 F4 rosbridge
세션이 초기화될 수 있다. 따라서 시나리오 본문만 설치하면 안 되고, 같은
stem의 EgoNetwork sidecar가 반드시 함께 있어야 한다.

```text
R_KR_PR_K-city_2025/
├── od_poc_01.json
└── EgoNetwork/
    └── od_poc_01_MN.json
```

`od_poc_01_MN.json`은 ROS로 파일 자체를 보내는 것이 아니다.
`/ScenarioLoad`에 `od_poc_01`과
`load_network_connection_data=true`를 보내면 MORAI가 로컬 폴더에서
두 파일을 함께 읽는다.

ScenarioLoad의 `delete_all`은 `false`로 보낸다. MORAI 문서상 이 상태에서
`load_ego_vehicle_data`, `load_surrounding_vehicle_data`,
`load_pedestrian_data`, `load_obstacle_data` 옵션을 선택적으로 적용한다.
`delete_all=true`로 보내면 실제 개인용 MORAI에서는 Ego 위치만 적용되고
주변 actor가 모두 제거되는 결과가 확인됐다.

제어 batch runner의 `configs/network/ego_network_1.json`은 대회용 UDP
profile이므로 이 ROS 캡처 PoC의 template으로 사용하지 않는다. 개인용
MORAI에서 ROS network를 연결한 상태로 저장한
`EgoNetwork/ros_path_probe_MN.json`의 Ego별 네트워크
`UNIQUEID=0`만 각 시나리오 sidecar에 사용한다.

Simulator Network `UNIQUEID=-100`은 sidecar에 넣지 않는다. 초기
route-follow PoC에서는 Ego-only sidecar로 p01~p03, 각 6프레임 연속
수집을 세 차례 성공했다. 이후 전체 `ros.json`을 sidecar로 복제했을 때는
ScenarioLoad마다 Simulator Network가 Disconnected로 바뀌고
`/Service_MoraiEventCmd`가 실패했다. 따라서 두 설정은 다음처럼 분리한다.

```text
시나리오 EgoNetwork/<stem>_MN.json
└── UNIQUEID=0만 포함

MORAI 시작 시 Simulator Network bootstrap
└── SaveFile/Network/26.R1.H3/ros.json의 UNIQUEID=-100 사용
```

`/Service_MoraiEventCmd`는 개인용 ROS profile에서도 `UNIQUEID=0`에
포함되어 있으므로 현재 route-follow/rolling 수집에는 Simulator Network가
필요하지 않다. Simulator Network는 `/ego_setting`, `/Service_MoraiSL`,
Sync Mode 등을 별도로 시험할 때만 전체 `ros.json`으로 Connect한다.

```text
Network Settings
→ Simulator Network
→ ROS
→ Load
→ SaveFile/Network/26.R1.H3/ros.json 선택
→ Connect
→ Status: Connected 확인
```

생성기는 전체 `ros.json`을 입력으로 받아도 `UNIQUEID=0`만 추출해
sidecar를 만들고, runner는 `-100`이 sidecar에 들어 있으면 실행을
거부한다. Ego-only sidecar의 필수 topic은 다음과 같다.

```text
/ScenarioLoad
/SaveSensorData
/Ego_topic
/Object_topic
```

manifest에는 추출된 Ego-only payload의 SHA256을 기록하고, 자동 캡처
노드는 실행 전에 설치된 모든 sidecar의 SHA256, 필수 ROS topic,
`UNIQUEID=0` 존재 및 `UNIQUEID=-100` 부재를 검사한다.

## 현재 PoC 장면

| 장면 | waypoint anchor | NPC | 보행자 | 장애물 |
|---|---:|---:|---:|---:|
| `p01` / `od_poc_01` | 500 | 4 | 3 | 2 |
| `p02` / `od_poc_02` | 2516 | 5 | 4 | 2 |
| `p03` / `od_poc_03` | 3500 | 4 | 5 | 3 |

모두 sunny, 13:00 메타데이터로 두었다. 현재 Internal Scenario JSON에서
날씨·시간 필드가 검증되지 않았으므로 MORAI UI에서 한 번 수동 설정한다.
이번 PoC의 목표는 환경 다양성이 아니라 자동 로드와 자동 캡처 성공 여부다.

## 한 시나리오당 몇 장을 저장할 것인가

PoC와 초기 대량 수집의 권장 기본값은 **한 시나리오당 6장**이다.

- Ego 목표 속도: 3.6 km/h = 1 m/s
- 프레임 간 이동거리: 0.6 m
- 총 이동거리: 약 3 m
- 장면당 캡처: 최초 위치 1장 + 이동 중 5장

정적 객체를 두고 20 Hz로 10~20장을 연속 저장하면 거의 같은 장면이 반복되어
학습 데이터 수만 부풀 가능성이 크다. 시나리오를 많이 만들 수 있다면
**많은 장면 × 5~8장**이 **적은 장면 × 20장**보다 낫다. 이후 가림,
근거리 소형 객체, 곡선처럼 변화가 큰 장면만 8~12장으로 늘린다.

예를 들어 5,000개 장면에서 6장씩 저장하면 30,000 프레임이다. 실제 split은
프레임이 아니라 시나리오 단위로 나누어 같은 정적 배치가 train과 val에
동시에 들어가지 않게 해야 한다.

## Ego 이동 방식

팀 제어 스택은 이 PoC에 사용하지 않는다. 자동화 스크립트가 공식
`2026_molit_comp_global_path.txt`를 따라 다음과 같이 제한된 이동만 수행한다.

1. `/Ego_topic`에서 현재 위치와 heading을 읽는다.
2. 가장 가까운 공식 waypoint와 3 m 전방 target을 찾는다.
3. 작은 pure-pursuit 조향과 `CtrlCmd.longlCmdType=2` 속도 명령을 20 Hz로 낸다.
4. 약 0.6 m 이동할 때마다 brake 명령으로 Ego를 정지시킨다.
5. 정지 상태에서 `/SaveSensorData`를 한 번 보내고 파일 저장이 완료된 뒤
   다시 출발한다.
6. 4 m 이동, 2.5 m 경로 이탈, 40° heading 오차, 충돌 또는 timeout이면
   즉시 brake 명령을 보낸 뒤 실패로 종료한다.

MORAI의 센서 파일 저장에는 약 1~2초가 걸릴 수 있다. 이 동안 마지막 속도
명령을 유지하면 한 번의 캡처 중에도 차량이 계속 움직여 0.6 m 간격을
건너뛸 수 있으므로, 현재 PoC는 각 프레임을 정차 상태에서 캡처한다.

## Ego 순간이동 방식

개인용 MORAI가 제공하는 `/ego_setting`
(`morai_msgs/MultiEgoSetting`)을 사용하는 `teleport` 모드도 지원한다.
현재 `/Ego_topic`의 `unique_id`를 그대로 사용하고, 공식 waypoint 위에서
다음 위치와 yaw를 계산해 Ego의 전역 자세를 변경한다.

```text
시나리오 로드 및 검증
→ Ego/Network는 보존하고 NPC·보행자·장애물만 로드
→ /ego_setting으로 Ego를 장면 anchor에 배치
→ 위치·yaw 및 객체 수 검증 후 f00 캡처
→ 공식 경로를 따라 0.6 m 앞의 자세 계산
→ /ego_setting 발행
→ /Ego_topic 위치·yaw 및 객체 수 재검증
→ 안정화 후 f01 캡처
```

`teleport` 모드의 ScenarioLoad 플래그는 다음과 다르다.

```text
delete_all=false
load_network_connection_data=false
load_ego_vehicle_data=false
load_surrounding_vehicle_data=true
load_pedestrian_data=true
load_obstacle_data=true
```

Ego와 Network를 다시 로드하면 Ego sidecar가 적용되는 동안 Simulator
Network가 뒤늦게 끊기는 현상이 확인됐다. 따라서 현재 연결된 Ego와 두
Network를 보존하고 주변 actor만 교체한다. 이후 Ego는 `/ego_setting`으로
각 시나리오의 anchor에 순간이동시킨다. 이 구조라면 Simulator Network의
수동 `Connect`는 batch 시작 전 한 번만 필요하다.

하지만 개인용 MORAI 실측 결과,
`load_ego_vehicle_data=false`에서는 주변 actor 플래그가 `true`여도
NPC·보행자·장애물이 모두 0개로 남았다. 즉 현재 버전에서는 이 선택적 로드
방식을 batch에 사용할 수 없다. 순간이동 기능 자체는 다음과 같이 분리해
검증한다.

이후 `roscore`와 `rosbridge`를 새로 시작한 상태에서는 actor-only load로
객체 수가 정상 적용되는 것까지 확인됐다. 하지만 그 직후
`/ego_setting`을 보내면 Ego 재구성과 함께 `/ScenarioLoad`,
`/SaveSensorData`, `/ego_setting`의 rosbridge subscriber가 모두
사라졌다. 반대로 사용자가 시나리오를 로드하고 두 Network를 다시 연결한
뒤 `--skip-scenario-load`를 사용하면 여러 번의 순간이동과 캡처가
성공했다. 따라서 현재 개인용 런처에서 다음 결합은 안정적인 batch
인터페이스로 간주하지 않는다.

```text
actor-only /ScenarioLoad
→ 즉시 /ego_setting
→ ROS 연결 유지 기대
```

### Network-preserving 후보의 실패와 폐기

`teleport → actor-only ScenarioLoad` 순서로 Ego와 Network를 보존하려는
`--network-preserving-batch`도 시험했으나, p01 actor-only load 직후
`/ScenarioLoad`, `/SaveSensorData`, `/ctrl_cmd`, `/ego_setting`이 모두
끊겼다. 따라서 이 옵션은 제거했다.

생산 후보는 이미 세 차례 성공한 다음 흐름으로 복원한다.

```text
UNIQUEID=0만 포함한 sidecar로 full ScenarioLoad
→ Ego·actor·Ego Network 자동 복구
→ ScenarioLoad·SaveSensorData·CtrlCmd·Event service 자동 복구
→ 매 장면 /Service_MoraiEventCmd로 ExternalCtrl·D 기어 확인
→ 3.6 km/h, 6프레임 캡처
→ 다음 장면
```

2026-08-07 복원 시험에서도 p01~p03 각 6프레임, 총 18프레임과 센서 파일
510개가 모두 검증됐다. 실행시간 68.839초는 과거 성공 실행
68.686~69.728초와 같은 범위다.

### 제어팀 trajectory_batch_runner와의 구조 비교

두 구현은 ScenarioLoad transport 외에도 세부 검증 범위가 다르지만 핵심
구조는 같다.

| 항목 | 제어팀 batch runner | MORAI OD 수집 |
|---|---|---|
| 시나리오 본문 | Ego와 actor 전체 로드 | Ego와 actor 전체 로드 |
| network flag | `load_network_connection_data=true` | 동일 |
| sidecar | `ego_network_1.json` 복제 | 개인용 ROS profile의 `UNIQUEID=0`만 추출 |
| Ego Network | 매 장면 sidecar로 복구 | 동일 |
| Simulator Network | 핵심 주행 경로에서 별도 사용 안 함 | route-follow/rolling에서 사용 안 함 |
| 제어 모드 | scenario/F4 UDP 설정에 의존 | Ego sidecar의 `/Service_MoraiEventCmd`로 매 장면 확인 |
| 적용 확인 | Ego 시작 위치 | fresh Ego 위치 + 객체 수 + ROS 연결 |
| 데이터 확인 | trajectory 평가 준비 | Camera/LiDAR/GT 파일까지 프레임별 검증 |

제어팀의 `configs/network/ego_network_1.json`에는 `UNIQUEID=0`과 `-100`이
모두 있지만, `-100`에는 한 개의 UDP 항목만 있다. 개인용 26.R1 ROS
profile의 `-100`에는 `/ego_setting`, Sync Mode, ScenarioLoad service 등
여러 Simulator service가 들어 있어 이를 sidecar로 복제했을 때 Connect
상태가 끊겼다. 따라서 이름이 같은 `-100`이라도 두 profile을 동일하게
취급하면 안 된다.

다중 시나리오 무인 실행은 일반 `/ScenarioLoad`와 `/ego_setting`을
연결하는 대신 Simulator Network의 Synchronous Mode
(`/SyncModeScenarioLoad`, `/SyncModeAddObject`,
`/SyncModeRemoveObject`) 지원 여부를 별도 PoC로 확인한 뒤 확장한다.

### Synchronous Mode PoC 결과와 제외 결정

2026-08-07 개인용 MORAI 런처에서 ROS Synchronous Mode를 별도로
시험했으나, 현재 데이터 수집 경로에는 사용하지 않기로 했다. 이 시험은
Scenario Runner를 실행한 것이 아니라 MORAI Simulator Network가 제공하는
ROS service와 topic만 사용했다.

확인한 인터페이스는 `/SyncModeCmd`, `/SyncModeWaitForTick`,
`/SyncModeCtrlCmd`, `/SyncModeSetGear`, `/SyncModeScenarioLoad`,
`/SyncModeInfo`이다. 센서별 Capture Mode를 끄고 Ego Network와 Simulator
Network를 다시 연결한 상태에서도 다음 두 종류의 비결정적 실패가
재현됐다.

1. `/SyncModeCmd`는 `result=True`를 반환하고 새로운 `master_id`를
   발급하지만, `/SyncModeInfo`가 계속 `status=True`, `frame=0`,
   `can_send_tick=False`에 머문다.
2. `roscore`, `rosbridge`, MORAI를 모두 재시작한 뒤에는
   `/SyncModeInfo`가 `status=False`, `frame=0`인 상태에서
   `/SyncModeCmd` service 호출 자체가 반환되지 않는 경우가 있다.

첫 번째 상태에서 gear와 정지 control을 frame 0에 넣어도 준비 상태가
바뀌지 않았다. `/SyncModeWaitForTick`은 `tick_status=False`,
`pause_status=False`, `frame=0`을 반환해 Scenario Editor 또는 일반 Pause
때문에 거절된 것도 아니었다. 또한 `/SyncModeInfo` publisher와 subscriber가
모두 연결되어 상태 메시지가 반복 수신됐으므로 단순 ROS topic 단절로
설명할 수 없다.

MORAI UI의 `Sensor Record`, 센서 `Synchronization Mode`, ROS External
Synchronous Mode는 서로 다른 기능이다. 현재 런처에서는 이들의 정확한
초기화 순서와 GT 포함 저장 동작을 재현 가능하게 확정하지 못했다.
따라서 대량 수집에서 frame 0 교착 또는 무응답 위험이 있는 Sync Mode를
사용하지 않는다.

현재 채택한 fallback은 이미 실측으로 통과한 다음 경로다.

```text
일반 /ScenarioLoad
→ network sidecar로 Ego Network 복구
→ AV-ExternalCtrl + D 기어
→ /ctrl_cmd 기반 제한형 route-follow
→ 정차
→ /SaveSensorData
→ Camera 4대와 LiDAR의 raw·semantic·instance·depth·3D GT 검증
```

`teleport`와 Sync Mode 코드는 후속 조사 재료로 남기되, 기본 실행과 대량
수집에는 `--motion-mode route-follow`을 사용한다. MORAI 업데이트나 공식
호출 예제가 확보되기 전까지 Sync Mode 실패를 우회하기 위한 추측성 재시도는
하지 않는다.

1. p01을 Ego·객체·Ego Network 포함 정상 로드한다.
2. 로드로 끊긴 Simulator Network를 UI에서 다시 `Connect`한다.
3. `--skip-scenario-load`로 현재 p01을 검증하고 순간이동만 수행한다.

`--skip-scenario-load`는 현재 장면 하나만 대상으로 사용할 수 있으며,
시나리오 파일명과 객체 수를 사용자가 먼저 맞춰야 한다.

수평 XY 오차 0.25 m, Z 오차 0.75 m, yaw 오차 5° 이내이고
NPC·보행자·장애물 수가 유지된 경우에만 캡처한다. XY와 Z를 분리한 이유는
`/ego_setting` 직후 MORAI가 차량 collider를 도로 표면에 안착시키면서
waypoint Z와 실제 차량 기준점 Z 사이에 수십 cm 차이가 생길 수 있기
때문이다. 도로상의 관측 위치를 결정하는 XY는 엄격하게 유지한다.
검증에 실패하면 추측으로 계속 진행하지 않고 즉시 중단한다.

캡처 검증은 센서 폴더가 생성됐는지만 보지 않는다. 각 프레임마다 다음
학습 산출물의 존재와 0보다 큰 파일 크기를 확인한 뒤에만 파일을 결과
폴더로 이동한다.

| 센서 | 필수 산출물 |
|---|---|
| CAMERA_1~4 | `Intensity.png`, `Semantic.png`, `Instance.png`, `Depth.png` |
| LIDAR_5 | `Intensity.bin`, `Semantic.bin`, `Instance.bin` |
| 객체가 있는 장면의 LIDAR_5 | `Instance.txt`, `Instance_8Points.txt` 추가 |

카메라 `BBox2D.txt`와 `BBox3D.txt`는 해당 카메라에서 유효한 객체가 없을
때 MORAI가 빈 파일조차 만들지 않으므로 필수 조건으로 두지 않는다. 생성된
경우에는 다른 산출물과 함께 보존된다. 실제 관측·필수·누락 산출물 목록은
`ros_capture_poc_report.json`의 `observed_artifacts`,
`required_artifacts`, `missing_artifacts`에 프레임별로 기록된다.

MORAI Simulator Network를 연결한 뒤 ROS subscriber를 먼저 확인한다.

```bash
# Simulator Network가 /ego_setting을 구독하는지 확인
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

rostopic info /ego_setting
```

`Subscribers`에 `/rosbridge_websocket`이 있어야 한다.

```bash
# p01에서 현재 자세 1장과 0.6 m 순간이동 후 1장만 안전하게 시험
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/poc_p01_teleport \
  --morai-scenario-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --scenario p01 \
  --motion-mode teleport \
  --captures-per-scene 2 \
  --maximum-motion-m 1.0
```

성공 로그에는 `EgoSetting published`, `Ego teleport verified`, `f00`,
`f01`, `auto-capture complete`가 모두 있어야 한다. 이 시험이 성공한
뒤에만 기본 6장 또는 여러 시나리오 batch로 확장한다.

이 방식은 “도로 전체를 안정적으로 주행하는 새 제어기”가 아니다. 데이터
관측점을 3 m 정도 바꾸기 위한 제한형 micro-motion이다. MORAI built-in
controller는 목적지와 내부 route 설정 의존성이 있어 대량 JSON마다 관리가
늘어나므로 첫 PoC에는 사용하지 않는다.

MORAI 버전에 따라 `morai_msgs/CtrlCmd`의 조향 필드가 다르므로 자동으로
호환 처리한다.

| 환경 | 조향 필드 |
|---|---|
| 대회용 메시지 | `steering` |
| 개인용 최신 메시지 | `front_steer`, `rear_steer` |

대회용 `steering`에는 rad 단위 조향각을 그대로 넣는다. 개인용
`front_steer`는 조향각 자체가 아니라 `[-1, 1]` 정규화 명령이므로,
계산한 조향각을 기본 최대 조향각 `36.25°`로 나누어 넣고
`rear_steer=0`으로 둔다. 최대 조향각은
`--personal-max-steering-angle-deg`로 변경할 수 있다. 실행 로그의
`CtrlCmd schema detected: steering field=...`로 실제 선택된 schema를
확인할 수 있다.

시나리오 로드 후 저장된 keyboard 제어 모드로 돌아갈 수 있으므로
`/Service_MoraiEventCmd`를 호출해 매 장면 AV-ExternalCtrl과
주행 기어 `D(4)`를 함께 선택한다. 생성 시나리오의 Ego 초기 기어도
`D(4)`로 저장해, 이전 상태의 `P` 기어 때문에 속도 명령이 무시되는 것을
이중으로 방지한다.
해당 service가 없는 MORAI 버전에서는 UI에서 AV-ExternalCtrl을 선택하고
`--skip-control-mode-service`를 사용하되, 다중 장면 자동화 전에 매번 제어
모드가 유지되는지 확인해야 한다.

## 1. 시나리오 생성 및 MORAI 폴더 설치

개인용 MORAI에서 현재 사용하는 K-City 2025 시나리오 폴더와, MORAI
UI에서 직접 저장한 전체 ROS profile을 지정한다. 생성기는 전체 profile을
sidecar로 그대로 복사하지 않고 `UNIQUEID=0`만 추출한다.

```bash
# PoC 시나리오 3개와 ROS network sidecar를 개인용 MORAI에 설치
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_ros_capture_poc_set.py \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --network-template \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Network/26.R1.H3/ros.json"
```

설치 확인:

```bash
# MORAI에 설치된 시나리오 본문과 ROS network sidecar 확인
cd ~/AjouNice2026

find "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  -maxdepth 1 -type f -name 'od_poc_*.json' -print | sort

find "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/EgoNetwork" \
  -maxdepth 1 -type f -name 'od_poc_*_MN.json' -print | sort
```

## 2. MORAI UI 준비

1. 개인용 MORAI에서 `R_KR_PR_K-city_2025` 맵을 연다.
2. sunny, 13:00로 설정한다.
3. 팀의 현재 Camera 4대 + 3D LiDAR 센서 구성을 적용한다.
4. Camera와 LiDAR에서 필요한 GT(Intensity, Semantic, Instance 등)를 켠다.
5. 모든 GT를 한 번에 저장하려면 각 센서의 **Capture Mode**를 켠다.
6. **Ego Network**의 ROS1 설정에서 다음 topic을 Connect한다.

```text
Subscriber: /ScenarioLoad, /SaveSensorData, /ctrl_cmd
Publisher:  /Ego_topic, /Object_topic, /CollisionData, /clock
```

7. Simulator Network는 현재 route-follow/rolling 수집에는 연결하지
   않아도 된다. `/ego_setting`, `/Service_MoraiSL`, Sync Mode를 별도로
   시험할 때만 전체 `ros.json`을 Load하고 Connect한다.

Capture Mode가 켜진 Camera/LiDAR는 평상시 Sensor List에서 비활성 또는
빨간 상태로 보일 수 있고 `/SaveSensorData` 순간에만 활성화된다. 이는 문서상
정상 동작이다.

## 3. ROS master

터미널 1:

```bash
# ROS master 실행
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
roscore
```

## 4. 연결 사전 점검

터미널 2:

```bash
# MORAI ROS topic과 제어 모드 service 연결 확인
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

rosparam set /use_sim_time false

rostopic info /ScenarioLoad
rostopic info /SaveSensorData
rostopic info /ctrl_cmd
rostopic echo -n 1 /Ego_topic
rostopic echo -n 1 /Object_topic
rosservice list | grep -F /Service_MoraiEventCmd
```

`/ScenarioLoad`, `/SaveSensorData`, `/ctrl_cmd`에는 MORAI subscriber가 각각
하나 이상 보여야 한다.

## 5. 저장 경로와 dry-run

`--capture-dir`에는 사람이 읽기 쉬운 실험 이름을 지정한다. 기본적으로
실행 시각이 `YYMMDD_HHMMSS_` 형식으로 자동 추가된다.

```text
입력: /home/luv/morai_cap/poc_multi_route_follow_v1
실제: /home/luv/morai_cap/260806_233523_poc_multi_route_follow_v1
```

이미 `260806_233523_` 형식의 prefix가 있으면 중복해서 붙이지 않는다.
기존처럼 정확히 입력한 경로를 사용해야 할 때만
`--no-timestamp-prefix`를 추가한다.

저장 경로는 MORAI가 실행되는 같은 PC의 절대 경로이며 legacy 제한 때문에
짧게 둔다.

```bash
# 짧은 외부 데이터 경로 생성 및 자동화 설정 정적 검사
mkdir -p /home/luv/morai_cap/poc

cd ~/AjouNice2026

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/poc \
  --morai-scenario-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --dry-run
```

이 검사는 ROS를 발행하지 않고 다음을 확인한다.

- 생성 원본과 MORAI에 설치된 scenario JSON이 같은지
- 각 scenario에 대응하는 `_MN.json`이 있는지
- 모든 `_MN.json`이 같은 ROS template SHA256인지
- 필수 ROS topic이 sidecar에 포함됐는지

센서 캡처의 기본 완전성 기준은 다음 5개 출력 디렉터리가 모두 새 파일을
생성하는 것이다.

```text
CAMERA_1
CAMERA_2
CAMERA_3
CAMERA_4
LIDAR_5
```

MORAI는 센서별 파일을 비동기로 저장하므로 첫 파일이 나타난 즉시 성공으로
처리하지 않는다. 필수 센서가 모두 나타나고 새 파일 목록이 기본 1초 동안
변하지 않아야 해당 capture를 완료로 판정한다.

개인용 MORAI에서는 `/SaveSensorData`의
`is_custom_file_name=true`와 custom `file_dir`을 사용했을 때 Camera는
저장되지만 3D LiDAR가 누락되는 동작이 실측됐다. 따라서 자동화 노드는
다음 방식으로 저장한다.

```text
is_custom_file_name=false
→ MORAI 기본 SaveFile/SensorData에 Camera 4대 + LiDAR 저장
→ 새 파일 묶음과 필수 센서 디렉터리 검증
→ 파일 목록이 1초 동안 안정될 때까지 대기
→ --capture-dir로 이동하면서 공통 capture_id로 이름 변경
```

예를 들면 MORAI가 생성한
`LIDAR_5/26-..._Intensity.bin`은 최종적으로
`LIDAR_5/r..._p01_f00_Intensity.bin`이 된다. 검증이 실패한 파일은
원인 분석을 위해 MORAI 기본 SensorData에 남기고 이동하지 않는다.

## 6. Phase A: 자동 로드 + 정지 1장

이 단계가 가장 중요하다. 이동을 전혀 하지 않고 `p01` 로드, Ego 위치,
객체 수, 실제 센서 파일 생성을 확인한다.

```bash
# p01 자동 로드 후 정지 상태 센서 캡처 1회 검증
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/poc \
  --morai-scenario-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --scenario p01 \
  --motion-mode stationary \
  --captures-per-scene 1
```

성공 기준:

```text
ScenarioLoad published: od_poc_01 (delete_all=false, load_network_connection_data=true)
MORAI ROS connections ready (scenario od_poc_01 network recovery)
Scenario verified: od_poc_01
Capture ...: files=N, sensors=CAMERA_1,CAMERA_2,CAMERA_3,CAMERA_4,LIDAR_5, relocated=N, verified=True
auto-capture complete
```

ScenarioLoad 직후 rosbridge의 MORAI client가 일시적으로 모두 끊겼다가
다시 연결되는 것은 정상이다. 자동화 노드는 wall clock 기준 최대 20초 동안
`/ScenarioLoad`, `/SaveSensorData`와 필요한 `/ctrl_cmd` 연결을 기다린다.
연결만 돌아온 것으로 성공 처리하지 않고, 로드 이후에 새로 수신한
`/Ego_topic` 위치와 `/Object_topic` 객체 수도 함께 검증한다.

실제 산출물:

```bash
# MORAI가 생성한 센서별 파일과 실행 보고서 확인
find /home/luv/morai_cap/poc -maxdepth 3 -type f | sort | head -n 80
cat /home/luv/morai_cap/poc/ros_capture_poc_report.json
```

이 단계가 실패하면 대량 실행으로 넘어가지 않는다.

## 7. Phase B: 한 장면 저속 이동 + 6장

MORAI 주변에 사람이 없고 차량이 움직여도 안전한 상태에서 실행한다.
팀 trajectory follower나 다른 `/ctrl_cmd` publisher는 모두 종료한다.

```bash
# p01에서 공식 waypoint를 따라 약 3 m 이동하며 6장 캡처
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/poc \
  --morai-scenario-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --scenario p01 \
  --motion-mode route-follow
```

다른 노드가 동시에 제어하지 않는지 확인:

```bash
# /ctrl_cmd publisher가 자동화 노드 하나뿐인지 확인
rostopic info /ctrl_cmd
```

## 8. Phase C: 3개 시나리오 자동 배치

Phase A와 B가 모두 통과한 뒤에만 실행한다.

```bash
# 세 장면을 순차 로드하고 각 장면에서 6장씩 자동 캡처
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/poc \
  --morai-scenario-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --motion-mode route-follow
```

총 기대 캡처 수는 18개다. 하나의 capture 명령은 Camera/LiDAR 각각의 여러
GT 파일을 만들므로 실제 파일 수는 18보다 훨씬 많다.

## 8.1 Phase D: 20개 시나리오 짧은 연속 수집

이 시험은 객체 배치 품질이나 주행 성능이 아니라 다음 항목만 빠르게
확인한다.

- 서로 다른 이름의 시나리오 20개 연속 ScenarioLoad
- 시나리오별 Ego Network sidecar 복구
- 시나리오별 객체 수와 Ego pose 확인
- 시나리오별 `/SaveSensorData` 1회
- Camera 4대와 LiDAR 필수 파일 저장

경로와 제어기의 영향을 배제하기 위해 모든 장면은 검증된 동일 anchor를
사용하고 seed만 바꾼다. 차량은 움직이지 않으며 장면당 1장만 저장한다.

```bash
# stress001~stress020과 sidecar를 생성하고 개인용 MORAI에 설치
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_ros_capture_poc_set.py \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --stress-count 20
```

MORAI·rosbridge 연결을 확인한 뒤 실행한다.

```bash
# 20개 시나리오에서 각각 정지 상태 1장씩 연속 캡처
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest dataset/morai_od/manifests/ros_capture_stress.json \
  --capture-dir /home/luv/morai_cap/stress20_stationary_v1 \
  --motion-mode stationary \
  --captures-per-scene 1 \
  --load-settle-sec 0.5 \
  --capture-stable-sec 0.3 \
  --continue-on-scene-error
```

성공 기준은 report의 장면 수와 capture 수가 모두 20이고, 모든 capture의
`verified`가 `true`이며 `missing_sensors`와 `missing_artifacts`가 비어
있는 것이다. 실패 장면은 report의 `status=failed`와 `error`에 남기고
다음 시나리오를 계속 처리한다. 이 시험이 통과해야 이동 캡처를 포함한
20~50장면 batch로 확장한다.

### 2026-08-07 최초 stress20 결과

첫 실행은 `stress001`부터 `stress016`까지 연속 성공하고
`stress017`의 장면 검증에서 중단됐다.

| 항목 | 결과 |
|---|---:|
| 연속 ScenarioLoad·capture 성공 | 16장면 |
| 저장된 capture | 16 |
| 저장 파일 | 452 |
| 0 byte 파일 | 0 |
| 필수 Camera·LiDAR 누락 | 0 |
| 중단 장면 | `stress017` |
| 중단 이유 | obstacle 예상 2개, 실제 `/Object_topic` 1개 |

`stress017`에서도 `/ScenarioLoad`와 `/SaveSensorData`의 rosbridge 재연결은
성공했다. 따라서 이 결과는 17번째에서 네트워크가 끊긴 증거가 아니라,
MORAI가 해당 장면의 actor 하나를 적용하지 않은 장면 무결성 실패다.

기존 구현은 한 장면 오류에서 전체 실행을 예외 종료해 최종 report도
남기지 않았다. 이후 `--continue-on-scene-error`를 추가했다. 이 옵션은
실패 장면을 `status=failed`와 오류 문자열로 기록하고 다음 manifest
장면을 계속 실행한다. 시작 시 필수 ROS 연결 자체가 없으면 batch 전체를
시작하지 않는 정책은 유지한다.

## 9. 실험용 rolling 처리량 비교

기존 `route-follow`은 각 캡처 전에 brake를 걸고 Camera 4대와 LiDAR의
필수 산출물이 모두 저장될 때까지 기다린다. 다음 두 모드는 데이터 품질보다
처리량을 먼저 측정하기 위한 실험용이며, 검증된 기본 수집 모드를 대체하지
않는다.

### rolling-safe

차량을 정차시키지 않지만 한 캡처의 파일 검증이 끝난 후 다음 캡처를
요청한다. `trigger_capture()`가 기다리는 동안 마지막 control 명령이
유지되므로 곡선 구간의 장거리 수집에는 사용하지 않는다. 저장 중 약
1~2 m를 이동할 수 있어 기본 4 m 제한보다 큰 이동 허용값을 명시한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/benchmark_p01_rolling_safe_v1 \
  --scenario p01 \
  --motion-mode rolling-safe \
  --captures-per-scene 6 \
  --capture-spacing-m 0.6 \
  --speed-kph 3.6 \
  --maximum-motion-m 12.0
```

### rolling-fast

1 m/s로 주행하며 0.6 m마다 `/SaveSensorData`를 보내고 개별 저장 완료를
기다리지 않는다. 6개 trigger를 모두 보낸 뒤 한 번만 정차하고 파일 쓰기가
안정될 때까지 기다린다. 목표 trigger rate는 약 1.67 Hz다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/benchmark_p01_rolling_fast_v1 \
  --scenario p01 \
  --motion-mode rolling-fast \
  --captures-per-scene 6 \
  --capture-spacing-m 0.6 \
  --speed-kph 3.6 \
  --maximum-motion-m 4.0
```

`rolling-fast`의 파일은 trigger별 GT 완전성을 주장하지 않고
`rolling_fast_raw/<scene id>/<sensor>/` 아래에 원래 MORAI 파일명으로
보존한다. 보고서에서 다음 값을 비교한다.

```text
scenes[].metrics.trigger_rate_hz
scenes[].metrics.end_to_end_capture_rate_hz
scenes[].metrics.trigger_span_sec
scenes[].metrics.drain_duration_sec
scenes[].metrics.observed_file_count
```

콘솔의 `Rolling-fast benchmark` 로그에도 trigger rate와 관측 파일 수가
출력된다. `trigger_rate_hz`는 주행 중 요청 속도이고,
`end_to_end_capture_rate_hz`는 시나리오 로드와 마지막 파일 drain까지
포함한 전체 처리량이다. 두 실험은 각각 새 MORAI 재시작까지 요구하지
않지만, 첫 실험 종료 후 차량과 시나리오가 초기 상태로 다시 로드됐는지
확인한 뒤 두 번째 명령을 실행한다.

### rolling-grouped

`rolling-fast`와 똑같이 주행 중 연속 trigger하고 마지막에 한 번만
기다린다. 차이는 MORAI가 한 번의 `SaveSensorData` snapshot에 공통으로
붙이는 timestamp를 기준으로 Camera 4대·LiDAR·GT 파일을 프레임별로
묶는다는 점이다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/p01_rolling_grouped_v1 \
  --scenario p01 \
  --motion-mode rolling-grouped \
  --captures-per-scene 6 \
  --capture-spacing-m 0.6 \
  --speed-kph 3.6 \
  --maximum-motion-m 4.0 \
  --capture-stable-sec 0.5
```

기존 p01~p03 rolling 결과를 사후 검사했을 때 장면마다 정확히 6개
timestamp가 있었고, 모든 timestamp에 `CAMERA_1..4`와 `LIDAR_5`의 필수
학습 산출물이 함께 존재했다. 따라서 실제 대량 수집은 매 프레임 정차
검사 대신 이 모드를 우선 사용한다.

검증을 완전히 생략하지는 않는다. 연속 trigger가 끝난 뒤 파일 집합이
한 번 안정될 때까지 기다리고, 각 timestamp 묶음의 필수 센서·GT 존재
여부만 검사한다. 6개 중 하나라도 불완전하면 그 장면의 원본을
`rolling_grouped_rejected/`에 보존하고 실패 처리한다. 누락 프레임은
버려도 되지만 서로 다른 시점의 image·point cloud·GT를 한 sample로
학습시키지는 않는다는 정책이다.

### 대기시간 축소 시험과 현재 채택 상태

대량 수집 시간을 줄이기 위해 다음 두 대기시간을 함께 축소할 수 있다.

- `--load-settle-sec 0.5`: 객체 수와 Ego 상태가 검증된 뒤 추가 대기를
  기본 2.0초에서 0.5초로 줄인다.
- `--capture-stable-sec 0.3`: 마지막 capture 뒤 새 파일 집합이 변하지
  않는지 확인하는 시간을 기본 1.0초에서 0.3초로 줄인다.

2026-08-07에 `p01`, 3.6 km/h, 0.6 m 간격, 6프레임
`rolling-fast`로 두 값을 동시에 시험했다.

| 항목 | 결과 |
|---|---:|
| trigger 수 | 6 |
| timestamp 그룹 수 | 6 |
| 전체 파일 수 | 162 |
| 그룹별 파일 수 | 27 |
| trigger span | 6.238초 |
| 마지막 file drain | 0.560초 |
| 장면 전체 시간 | 10.677초 |
| end-to-end 처리량 | 0.562 frame/s |

같은 p01의 기존 기본 대기시간 결과와 비교하면 다음과 같다. 두 실행의
실제 주행 속도와 시나리오 로드 응답시간이 완전히 같지는 않으므로
`load-settle-sec`의 효과만 분리한 엄밀한 A/B 측정은 아니다.

| 항목 | 기본 2.0/1.0초 | 축소 0.5/0.3초 | 변화 |
|---|---:|---:|---:|
| trigger span | 6.087초 | 6.238초 | +0.151초 |
| 마지막 file drain | 1.210초 | 0.560초 | -0.650초 |
| 장면 전체 시간 | 10.875초 | 10.677초 | -0.199초 |
| end-to-end 처리량 | 0.552 frame/s | 0.562 frame/s | +1.9% |

`capture-stable-sec` 축소는 drain 감소로 직접 관찰됐다. 반면 전체 시간은
시나리오 로드와 차량 가속 변동이 섞여 이 한 쌍만으로 1.5초 절감을
보장할 수 없다.

각 timestamp 그룹에서 Camera 1~4의 Intensity·Semantic·Instance·Depth와
관측 객체가 있는 카메라의 BBox2D·BBox3D, LiDAR의
Intensity·Semantic·Instance·Instance.txt·Instance_8Points.txt가 모두
확인됐다. 따라서 이 한 장면에서는 축소 조합으로 인한 파일 누락이
관찰되지 않았다.

Ego Network만 담은 sidecar를 복원한 뒤 p01~p03 연속 시험도 통과했다.
3개 장면 모두 6개 timestamp 그룹과 Camera 1~4·LiDAR 5의 필수 산출물이
확인됐고, 0 byte 파일은 없었다.

| 항목 | route-follow 기준 | rolling-fast 0.5/0.3초 |
|---|---:|---:|
| 장면 수 / 캡처 수 | 3 / 18 | 3 / 18 |
| 전체 센서 파일 수 | 510 | 510 |
| 누락 센서·필수 산출물 | 0 | 0 |
| 전체 실행시간 | 68.839초 | 34.666초 |
| 시간 단축 | - | 49.6% |

바로 이어서 같은 조건을 반복했을 때 p02에서 `/Ego_topic`이 갱신되지
않고 `/Service_MoraiEventCmd` 호출도 응답하지 않아 실행이 멈추는 현상이
한 차례 발생했다. 이는 캡처 대기시간이나 파일 누락 문제가 아니라 오래
연결된 MORAI·rosbridge의 제어 통신 상태 문제로 분류한다. 자동화 노드는
이후 제어 모드 service 응답 제한시간과 Ego heartbeat 확인을 추가해,
같은 상태에서 무기한 정지하지 않고 명확한 오류로 종료하도록 보완했다.

따라서 현재 정책은 다음과 같다.

1. 스크립트 기본값 `2.0초/1.0초`는 유지한다.
2. 초기 빠른 데이터 수집에서는 아래 `0.5초/0.3초` 조합을 사용할 수
   있다. p01~p03 18프레임의 1차 통과 근거가 있다.
3. 장시간 무인 수집 전에는 소규모 묶음으로 나눠 실행하고, 묶음 사이에
   `/Ego_topic` 갱신과 제어 service 응답 여부를 확인한다.
4. 수백~수천 장 기본값으로 확정하기 전에는 서로 다른 장면에서
   timestamp 그룹 수, Camera 4대, LiDAR 및 BBox/GT 누락률을 표본
   검사한다.
5. MORAI 제어 service·heartbeat 오류는 센서 누락과 별도로 집계한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/rolling_fast_fast_wait_v1 \
  --scenario p01 \
  --motion-mode rolling-fast \
  --captures-per-scene 6 \
  --capture-spacing-m 0.6 \
  --speed-kph 3.6 \
  --maximum-motion-m 4.0 \
  --load-settle-sec 0.5 \
  --capture-stable-sec 0.3
```

### 장면당 프레임 수 정책

현재 PoC와 초기 학습 데이터 수집은 **장면당 6프레임**을 유지한다. 센서
동기화, 수집 안정성, 장면 내 시점 변화량을 먼저 검증하기 위해서다.

향후 동일한 총 프레임 예산에서는 장면당 3~4프레임으로 줄이고 시나리오
수를 늘리는 방안을 비교한다. 예를 들어 `180장면 × 6프레임`과
`360장면 × 3프레임`은 모두 1,080프레임이지만, 후자는 객체 종류·배치·
가림·거리·방향·날씨의 다양성을 늘리기 유리할 가능성이 크다. 다만 이
변경은 첫 파인튜닝 성공과 100장면 수집 안정성 확인 이후 별도 실험으로
결정하며, 현재 수집 명령의 `--captures-per-scene 6`은 변경하지 않는다.

### 전방 무객체 7.2 km/h benchmark

`speed01` (`od_speed_clear_01`)은 p01과 같은 공식 경로 anchor를 사용하지만
NPC·보행자·장애물을 모두 제거한 속도 전용 장면이다. 기존 p01~p03 기본
batch에는 포함되지 않으며 `--scenario speed01`로 명시할 때만 실행된다.

먼저 새 시나리오와 ROS network sidecar를 개인용 MORAI 폴더에 설치한다.

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_ros_capture_poc_set.py \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --network-template \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Network/26.R1.H3/ros.json"
```

설치 후 7.2 km/h 명령, 0.6 m 거리 trigger로 6프레임을 시험한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --capture-dir /home/luv/morai_cap/benchmark_speed01_rolling_fast_2ms_v1 \
  --scenario speed01 \
  --motion-mode rolling-fast \
  --captures-per-scene 6 \
  --capture-spacing-m 0.6 \
  --speed-kph 7.2 \
  --maximum-motion-m 4.0
```

이 장면은 actor GT 생성량과 검출 품질을 평가하지 않는다. 실제 평균 Ego
속도, `trigger_rate_hz`, `end_to_end_capture_rate_hz`, 6개 timestamp
그룹과 Camera 4대·LiDAR 파일 완전성만 기존 3.6 km/h 결과와 비교한다.

## 실패 해석

| 증상 | 의미와 조치 |
|---|---|
| ScenarioLoad subscriber 없음 | MORAI ROS Subscriber 설정 확인 |
| `_MN.json` 없음 또는 SHA256 불일치 | 생성·설치 명령을 다시 실행하고 ROS template 경로 확인 |
| ScenarioLoad 직후 ROS 연결 복구 timeout | sidecar가 ROS bridge profile인지, MORAI port 9090 연결 상태인지 확인 |
| Ego는 이동했지만 객체 수가 모두 0 | `delete_all=false`인지 확인; 개별 actor load 옵션은 모두 true여야 함 |
| Ego 위치가 기대값과 다름 | 현재 맵 폴더, file stem, ScenarioLoad ego 옵션 확인 |
| 객체 수 불일치 | 시나리오 로드 실패 또는 actor active 상태 확인 |
| 새 파일 0개 | Capture Mode, GT 설정, MORAI 기본 SensorData 경로 권한 확인 |
| Camera는 저장됐지만 `LIDAR_5` 누락 | MORAI Sensor List에서 3D LiDAR Capture Mode 확인; custom 경로가 아니라 기본 저장 방식인지 확인 |
| Event service 없음 | service를 켜거나 UI 수동 전환 + skip 옵션 |
| 경로/heading safety abort | 이동을 중단한 정상 보호 동작; anchor·heading 검토 |
| collision abort | 해당 장면 폐기 또는 객체 clearance 수정 |
| Sync Mode가 `frame=0`, `can_send_tick=false`에 고정 | 현재 개인용 런처에서 재현된 미해결 교착; Sync Mode를 사용하지 말고 검증된 `route-follow` 경로 사용 |
| `/SyncModeCmd` 호출이 반환되지 않음 | Simulator Network service가 불안정한 상태; 대량 수집에서는 Sync Mode를 제외하고 일반 `/ScenarioLoad`·`/SaveSensorData` 사용 |

`--allow-unverified-capture`는 MORAI 버전별 저장 동작을 진단할 때만
사용한다. 대량 수집에서는 파일 생성이 검증되지 않은 캡처를 성공으로
간주하면 안 된다.

## PoC 이후

이 PoC가 성공하면 다음 순서로 확장한다.

1. 객체 유형 catalog와 차선·보도·횡단보도 제약 정교화
2. 장면별 seed와 rejection reason을 manifest에 기록
3. 날씨 2종 × 시간 3종 batch 분리
4. 시나리오당 5~8장 기본, 어려운 장면만 증량
5. 센서 파일 completeness, timestamp, GT 개수 자동 검사
6. 시나리오 단위 train/val/test split
