# MORAI full stack 터미널별 실행법

## 범위

이 문서는 하나의 shell wrapper가 아니다. 각 구성요소를 별도 터미널에서 실행해
문제가 생긴 계층을 바로 확인할 수 있도록 정리한 운영 runbook이다.

기준 구성은 다음과 같다.

```text
MORAI sensors/Competition (production: no Ego GT)
  -> UDP bridge
  -> Base5 E12 BEVFusion
  -> EKF multi-object tracking
  -> UNKNOWN stationary policy
  -> Map-CV prediction
  -> Autoware planning
  -> PP-PI controller
  -> MORAI /ctrl_cmd UDP
```

실제 대회 client PC 연결 전 점검 순서와 Go/No-Go 기준은
[`260915_field_inspection_checklist.md`](../competition/260915_field_inspection_checklist.md)를
함께 사용한다.

현재 production detector는 전체 30,750장 Base5 E12이며 class threshold는
Vehicle/Pedestrian/Obstacle `0.40/0.10/0.10`이다. 실제 모델 기준은
[모델 CURRENT](../bevfusion/CURRENT.md), 최초 weights·증분 빌드는
[배포 안내](../bevfusion/BASE5_BEVFUSION_PRODUCTION_HANDOFF.md)를 따른다.
E25 명시적 복귀는 `bevfusion --model e25`이며 실제 24GB 풀스택 점검은 별도다.

프로젝트 용어에서 `MRP`는 2026 개인용 launcher, `MR`은 2025 대회용 launcher를
뜻한다. 개인 개발은 MRP, 대회 환경 재현과 현장 실행은 MR을 기준으로 하되,
Sensor/Ego Network IP·port는 선택한 launcher와 repository JSON이 일치해야 한다.

## 최초 확인

모든 터미널에서 같은 repository와 ROS master를 사용한다.

```bash
cd "$HOME/AjouNice2026"
test -f scripts/workspace/source_aw149.sh
test -f base_ws/install_aw149/setup.bash
test -f planning_ws/install_isolated/setup.bash
test -f kcity_map/lanelet_map/lanelet2_map_v99.osm
```

MORAI에서는 다음을 확인한다.

1. MORAI의 공식 K-City 맵과 원하는 competition scenario를 연다. 알고리즘 지도는 Lanelet v99다.
2. Sensor Network와 Ego Network를 현재 PC 설정에 맞게 연결한다.
3. 실제 제어 전에 Ego control mode를 `AV-ExternalCtrl`로 둔다.
4. 차량은 Route A 공식 시작 위치에서 정지 상태로 둔다.

로드할 현장점검 원본의 정확한 경로와 IP·Ego 미사용 기준은
[현장 파일 안내](../../configs/morai_field_reference/README.md#합의한-대회-실행-기준)를 따른다.

## 마지막 대비 브랜치: 한국어 1초 요약 화면

`competition/final-readiness-261028`에서 추가한 **명시적 선택 기능**이다.
기존 alias·노드 로그 레벨·주행 정책은 바꾸지 않는다. `field`만 앞에 붙이면
기존 명령과 인자를 그대로 실행하면서 stdout/stderr 원문은 파일에 남기고 화면은 요약한다.
기존 `명령 | field_log_tee 이름`과 **동시에 사용하지 않는다**.

각 터미널에서 `ajou` 후 아래 한 줄씩 실행한다. Terminal 1이 먼저이며 제어는 마지막이다.

| 터미널 | 실행 명령 |
|---|---|
| 1 ROS·새 공유 로그 세션 | `field rc` |
| 2 대회 LAN 통신 | `field udp_lan enable_ego_vehicle_status:=false` |
| 3 센서 좌표계 | `field sensing_lan` |
| 4 공식 START 측위 | `field localization initial_position_source:=official_start` |
| 5 E12 객체 검출 | `field bevfusion` |
| 6 추적·지도 예측 | `field tracking` |
| 7 신호등 | `field traffic` |
| 8 판단 | `field planning` |
| 9 지도 화면 | `field planviz show_morai_gt:=false` |
| 10 제어: 궤적·제어권·안전 확인 후에만 | `field traj_fol_a` |

로컬 MORAI는 `field udp`, `field sensing`으로 바꾼다. **공식 START 초기화를 중간
checkpoint에 적용하지 않는다.** 실행 중인 스택을 이 기능이 자동 교체/재시작하지 않는다.
기존 터미널을 재시작해야 새 표시가 적용되며 제어 중 임의 재시작하지 않는다.

- 보통 1초에 한 줄: 해당 구간의 오류/주의 횟수와 가장 중요한 로그를 표시한다.
  새 오류 종류는 즉시 알리되 한 구간 최대3종이다. 반복 오류는 횟수로 합친다.
- 오류 뒤 INFO가 와도 **누적오류·마지막 오류 시각**을 남긴다(횟수는 오류 로그 행 수).
  이는 오류 이력이며
  지속 고장/복구 판정은 아니다. 새 실행에서 초기화한다.
- 모르는 경고/오류는 원문 일부를 그대로 표시한다. 번역 규칙 밖의 의미를 추정하지 않는다.
  INFO는 최신 한 항목만 보이므로 상세 진단은 원문을 확인한다.
- `출력 대기·새 로그 없음`은 토픽 정상 판정이 아니다. Hz·지연·제어 상태는 별도다.
  신호등 `OFF/UNKNOWN`도 원거리·통과후·정상 미션 외 상황을 포함하므로 단독 고장으로 표시하지 않는다.
- 시작 전에 원문 파일을 못 열면 명령을 시작하지 않는다. **시작 후** 디스크 오류는
  `원문 저장 실패 지속`을 남기고 실제 스택은 유지한다. 그 이후 원문은 보존되지 않는다.
  자식이 정상 종료했더라도 이 경우 wrapper는 기록 실패 코드2를 반환한다.
- Ctrl-C/TERM은 **이번 실행이 만든 프로세스 그룹만** 종료 요청한다. 기본20초 뒤에도
  종료하지 않으면 해당 그룹만 강제 종료한다. 차량 정지·P를 보장하는 비상정지 장치는 아니다.
  기존처럼 제어부터 종료하고, 살아 있는 다른 스택에 일괄 `pkill`하지 않는다.
  wrapper가 터미널 종료 SIGHUP을 받으면 Ctrl-C와 같은 종료 요청을 전달한다.

원문 경로는 시작 시 출력한다. 같은 공유 세션 아래
`artifacts/field_logs/<세션>_console/<명령시각>_<구성요소>_raw.log`에 저장하며
매번 새 파일을 사용한다. Docker 내부 출력 전체·ROS bag·`/rosout` 전체를 대신하지 않는다.

```bash
field_log_list                       # 현재 공유 세션의 원문 파일 확인
tail -f /시작할_때_표시된/원본_raw.log # 상세 로그가 필요할 때 별도 터미널
python3 scripts/runtime/field_console.py demo  # ROS를 시작하지 않는 모의 출력
python3 scripts/evaluation/render_field_console_preview.py # 선택: PNG 예시, Pillow 필요
```

기본 화면 폭120칸이다. 좁은 터미널은 실행 전에 `export AJOUNICE_FIELD_LOG_WIDTH=90`으로
줄인다(최소60). 기본1초 주기는 `AJOUNICE_FIELD_LOG_INTERVAL`로 바꿀 수 있다(최소0.05초).
터미널을 요약하지 않으려면 기존 alias를 그대로 실행하면 된다. 재빌드·가상환경은 필요 없다.

### 현장에서 자주 확인할 메시지의 의미

아래는 **현재 소스의 오류 서명별 대응**이며 발생 빈도 통계나 자동 복구 기능이 아니다.

| 요약 | 우선 확인 |
|---|---|
| 추론 서버 요청/센서 전처리 실패 | BEVFusion Docker·가중치·서버, 입력/예외 원문 |
| 카메라·LiDAR 시각 불일치·입력 폐기 | sensor stamp·카메라 입력·부하; 단순 GPU 속도 문제로 단정 금지 |
| 검출 발행 중복·실행 거부 | 기존 detector publisher 확인; 임의로 안전 검사를 해제하지 않음 |
| 신호 CNN 실행 불가/가중치 없음 | 신호등 설치기·관리 Python·모델 경로 확인 |
| 판단 경로 없음/입력 대기 | route·측위·perception·`/planning/trajectory` 확인 |
| 입력 오래됨 | 실제 입력 Hz/age, 차량 상태·trajectory·제어 토픽 확인 |
| GPS 높이 유지·XY 추정 계속 | 음영의 의도된 z 유지일 수 있음; XY 정확도 인증은 아님 |
| TRT 미사용·동일 모델 일반 추론 | 동일 모델 fallback이므로 인식 중단은 아님; 속도 확인 |
| 신호 미수신·기존 GO 우회 중 | 녹색을 읽었다는 뜻 아님; 기존 fail-open 경로 확인 |
| 포트 점유/ROS master/지도 오류 | 중복 실행·ROS 환경·v99 준비 및 원문 확인 |
| 원문 저장 실패 지속 | 디스크 공간/권한 확보; 실행은 계속되지만 이후 원문 누락 |

Controller는 입력 stale/frame 불일치에 **stdout 없이 정지 명령을 낼 수도 있다**.
이 화면만으로 정지 원인을 모두 알 수는 없다. `/control/autoware/failure_reason`,
`/ctrl_cmd`, state/trajectory의 freshness와 rosbag도 함께 확인한다.
이 브랜치의 모의/오프라인 로그 검사는 실대회 LAN 폐루프 검증이 아니다.

검증(2026-10-28): 모의 실행·분류·종료·디스크/화면 장애 격리29개와 관련 신호등
launch/배포 계약을 합쳐50개 통과했다. 기존 planning2416행·traffic103행 로그를
오프라인 처리한 raw 파일은 각각 원본 SHA256과 일치했다. 실제 스택을 새로 시작하거나
중단하지 않았으며 출력 예시 PNG는 모의 로그임을 명시한다.

## 대회 현장 권장: alias + `tee` 로그 세션

`tee`를 사용하면 화면 출력과 파일 저장이 동시에 이루어진다. 모든 터미널은 같은
로그 세션을 사용해야 하므로 Terminal 1에서 반드시 `field_log_new`를 먼저 실행한다.
그 외 터미널에서 세션이 없거나 다른 로그 경로가 잡히면 `field_log_tee`가 오류를 내고
pipeline을 실패 처리하므로 로그 누락을 바로 확인할 수 있다.

로그 파일명은
`<세션시각>_<구성요소>_<해당명령시작시각>.log` 형식이다. `field_log_tee`는
`tee -a`를 사용하지 않으며 호출할 때마다 새 파일을 만든다. 같은 구성요소를 같은 초에
다시 실행해도 `_01`, `_02`가 붙으므로 기존 로그를 덮어쓰거나 이어붙이지 않는다.

각 터미널은 `~/.bashrc`에 등록한 `ajou` 함수로 준비한다. 이 함수는
repository root로 이동하고 `AJOUNICE_ROOT`를 설정한 뒤 AW149 overlay와
`docs/common_alias.txt`를 로드한다. 새 shell에서 `type ajou`로 먼저 확인한다.

```bash
ajou
set -o pipefail
```

`ajou` `command not found`가 나오는 PC에서만 다음을 `~/.bashrc`에 한 번
등록하고 새 터미널을 연다.

```bash
ajou() {
    cd "$HOME/AjouNice2026" || return
    export AJOUNICE_ROOT="$HOME/AjouNice2026"
    source "$AJOUNICE_ROOT/docs/common_alias.txt"
}
```

### Terminal 1: 로그 세션 생성 + ROS master

반드시 가장 먼저 실행하고 전체 실행이 끝날 때까지 유지한다.

```bash
ajou
set -o pipefail

field_log_new
rc 2>&1 | field_log_tee roscore
```

`field_log_new`는 `artifacts/field_logs/<YYMMDD_HHMMSS>_session.txt`에 Git SHA,
host, ROS 환경을 기록하고 `/tmp/ajounice_field_log_prefix`를 원자적으로 갱신한다.
같은 초에 세션을 다시 만들어도 기존 로그를 덮어쓰지 않는다.

### Terminal 2: 현장 LAN UDP gateway

```bash
ajou
set -o pipefail

udp_lan enable_ego_vehicle_status:=false 2>&1 | field_log_tee udp_lan
```

camera/IMU/GPS 수신, Velodyne LiDAR 변환, Competition 상태 수신과 `/ctrl_cmd` UDP
송신을 담당한다. 이 대회 명령은 `/Ego_topic` 수신·광고를 끈다.

### Terminal 3: sensing static TF + 차량 모델

```bash
ajou
set -o pipefail

sensing_lan 2>&1 | field_log_tee sensing
```

`udp_lan`에는 sensor static TF가 포함되지 않으므로 생략하면 안 된다. 이 launch의
`enable_ego_tf` 기본값은 `false`라서 Terminal 4의 localization TF와 중복되지 않는다.
`sensing_lan`은 UDP gateway와 같은 competition sensor JSON에서 TF 위치를 읽는다.

### Terminal 4: localization + TF

```bash
ajou
set -o pipefail

localization initial_position_source:=official_start 2>&1 | field_log_tee localization
```

이 alias는 `localization_gnss_imu_gt_ctra.launch`를 실행한다. `sensing_lan`이 sensor
static TF를 담당하므로 localization의 `publish_sensor_tf` 기본값은 `false`로 유지한다.
위 명령은 공식 START 출발용이다. 개발 alias의 기본 Ego 위치를 사용하지 않으며,
`Published GNSS+IMU initial pose from official_start`와 초기 pose 발행을 확인한다. 중간 위치·checkpoint에 강제로 적용하지 않는다.

### Terminal 5: BEVFusion detection

```bash
ajou
set -o pipefail

bevfusion 2>&1 | field_log_tee bevfusion
```

이 alias는 Docker inference server와 ROS bridge의 생명주기를 함께 관리한다. canonical
detection topic에 PointPainting/FastBEV/다른 bridge publisher가 있으면 안전하게
거부한다. 다만 Docker container 내부의 상세 stdout 전체는 이 파일에 계속 복제되지
않으므로 필요할 때만 별도 터미널에서 아래 진단 로그를 추가한다.

```bash
ajou
set -o pipefail

docker compose -f docker/bevfusion/compose.yaml logs --follow --tail=100 inference \
  2>&1 | field_log_tee bevfusion_inference
```

### Terminal 6: tracking + map-based prediction

```bash
ajou
set -o pipefail

tracking 2>&1 | field_log_tee tracking
```

### Terminal 7: traffic-light recognition

```bash
ajou
set -o pipefail

traffic 2>&1 | field_log_tee traffic
```

### Terminal 8: planning

```bash
ajou
set -o pipefail

planning 2>&1 | field_log_tee planning
```

`planning`은 map, route requester와 planning graph만 실행한다. 이름과 달리 trajectory
follower/control은 포함하지 않는다.

### Terminal 9: 공용 Planning RViz

```bash
ajou
set -o pipefail

planviz show_morai_gt:=false 2>&1 | field_log_tee planviz
```

실대회 production에서는 GT overlay를 끈다. MORAI 개발 중 `/Object_topic` 비교가
필요할 때만 `show_morai_gt:=true`로 바꾼다.

### Terminal 10: control — 안전 확인 후에만 실행

처음에는 이 터미널을 실행하지 않고 `/planning/trajectory`와 validation 상태까지
확인한다. 차량 제어권과 비상정지 담당을 확인한 뒤에만 실행한다.

```bash
ajou
set -o pipefail

traj_fol_a 2>&1 | field_log_tee control
```

`traj_fol_a`만 `/planning/trajectory`를 받아 `/ctrl_cmd`를 만드는 현재 Autoware용
controller다. `traj_fol`이나 다른 `/ctrl_cmd` publisher를 동시에 실행하면 안 된다.

### 로그 확인과 topic-rate snapshot

별도 터미널에서 현재 세션 파일을 확인한다.

```bash
ajou

field_log_list
```

전체 stack이 올라온 뒤 topic rate를 같은 30초 구간에서 확인하고 화면과 세션 로그에
동시에 남기려면 다음을 실행한다.

```bash
set -o pipefail
bash scripts/runtime/record_field_topic_rates.sh 30 \
  2>&1 | field_log_tee topic_rates
```

`tee` 로그는 terminal stdout/stderr만 보존한다. ROS 메시지 자체, 센서 원본, `/tf`,
`/rosout`의 완전한 사후 재생을 보장하지 않으므로 rosbag을 대체하지 않는다. 실제
주행 데이터 기록은 현장점검 체크리스트의 `모든 로그 저장` 절차를 별도로 따른다.

종료는 control(Terminal 10)을 가장 먼저 끄고, planning/perception/localization,
sensing/UDP 순서로 내린 뒤 roscore를 마지막에 종료한다.

## 로컬 localization 실시간율 경량 검증

한 PC에서 MORAI와 폐루프 localization/control을 함께 실행할 때만 사용하는
비-production 구성이다. MORAI에는
`configs/sensors/legacy/Sensor_localization_gt_lightweight.json`을 로드한다. 이 파일은 MR과 MRP의
`SaveFile/Sensor` 양쪽에 둘 수 있으며 GPS와 IMU만 생성한다.

### 고정 시나리오와 시험 계약

MR에서 다음 시나리오를 연다.

```text
0_localization_full_route_no_actors
```

tracked 원본은
`configs/scenario/localization_evaluation/localization_full_route_no_actors.json`이다.
MR `SaveFile/Scenario/R_KR_PR_K-city_2025`에서는 목록 상단에 보이도록 로컬
복사본의 파일명에만 `0_`를 붙였다. 같은 방식으로 샘플 시나리오와 Base3 정적
장애물 벤치마크도 각각 `0_2026_molit_comp_sample_scene.json`,
`0_bench_static_obstacle_base3_assets_raw.json`으로 표시된다. Git에 있는 원본
시나리오 이름이나 JSON 내용은 바꾸지 않는다. 이 MR 로컬 파일명은 Git pull만으로
다른 PC에 생성되지 않는다.
원본 sample의 Ego 시작점, 전체 코스와 GPS 음영 영역은 유지하고 NPC, 보행자,
장애물과 모든 actor spawn을 제거했다. Sensor는
`configs/sensors/legacy/Sensor_localization_gt_lightweight.json`, Ego Network는
`ego_network_competition_local.json`을 로드한다. control mode는
`AV-ExternalCtrl`로 설정한다.

이 절차는 planning과 TF에는 기존 Global state를 유지하면서 controller만 production
Smooth local control state(`tau=0.5/0.5 s`)를 받는다. `traffic`과 detector는 실행하지
않고 planner의 TrafficLight plugin도 끈다.

### Terminal 0: MR launcher와 시나리오

```bash
mr
```

MR UI에서 위 시나리오, lightweight Sensor, local Ego Network를 차례로 로드한다.
아직 Route A를 누르거나 차량을 출발시키지 않는다.

### Terminal 1: ROS master

```bash
ajou
rc
```

### Terminal 2: 경량 UDP gateway

```bash
ajou
udp_localization_light
```

### Terminal 3: 경량 sensing/TF

```bash
ajou
sensing_localization_light
```

### Terminal 4: MR production localization

```bash
ajou
localization
```

`/localization/kinematic_state`는 Global planning/TF용이며,
`/localization/control_kinematic_state`만 production controller 입력이다.

### Terminal 5: MORAI GT object adapter

```bash
ajou
od_gt
```

시나리오에 객체가 없어도 빈 detection message를 계속 공급해 planning의 perception
입력 계약을 유지한다.

### Terminal 6: tracking + Map-CV

```bash
ajou
tracking
```

### Terminal 7: 신호등 없는 planning

```bash
ajou
planning
```

`traffic` alias는 실행하지 않는다. 기존 competition fail-open 기본값에서 신호 미수신·timeout은 GO다.
새 alias/인자는 필요 없다. 신선한 적색·UNKNOWN에는 기존 정지 판단을 유지하며, 노드 다운도 timeout GO가
될 수 있다는 정책 경계는 구분한다. 기존 `enable_traffic_light_planner:=false` 강제 우회 인자도 그대로 지원한다.

### Terminal 8: Planning RViz

```bash
ajou
planviz show_morai_gt:=true
```

차량, lanelet map, localization과 route 대기 상태가 정상인지 확인한다.

### Terminal 9: localization 평가 rosbag

```bash
ajou
bash scripts/rosbag/record/rosbag_for_localization_stability_eval.sh \
  smooth_tau0p5_full_route_no_actors
```

`Recording to`가 나온 뒤에만 다음 단계로 간다. 출력은
`rosbags/eval/localization/local_mr_stability/<시각>_smooth_tau0p5_full_route_no_actors.bag`
및 같은 이름의 metadata 파일이다. Camera, LiDAR, PointCloud는 기록하지 않는다.

### Terminal 10: controller

Route A를 먼저 요청해 RViz에 유효한 `/planning/trajectory`가 생기는지 확인한 뒤
controller를 실행한다.

```bash
ajou
traj_fol_a
```

차량이 출발하지 않으면 controller 인자를 바꾸기 전에 다음을 확인한다.

```bash
rostopic hz /localization/kinematic_state
rostopic hz /localization/control_kinematic_state
rostopic hz /planning/trajectory
rostopic hz /ctrl_cmd
```

완주 후 Terminal 10의 controller를 가장 먼저 끄고 Terminal 9의 recorder를
`Ctrl-C`로 정상 종료한다. 그 다음 planning부터 역순으로 종료한다.

### Global-state baseline 재현

동일 시나리오에서 baseline을 추가로 녹화할 때만 Terminal 4와 Terminal 10을 다음처럼
바꾼다. 나머지 조건은 전부 유지한다.

```bash
# Terminal 10
ajou
traj_fol_global
```

bag tag는 `global_full_route_no_actors`를 사용한다.

### Quantized-GNSS 후보 폐루프 재현

Production 기본값을 바꾸지 않고 quantization-aware controller state의 마지막 폐루프
gate를 수행할 때만 Terminal 4를 다음으로 바꾼다. Terminal 10은 production과 같은
`traj_fol_a`를 사용한다.

```bash
ajou
roslaunch bringup localization_gnss_imu_gt_ctra.launch \
  gt_source:=competition \
  use_heading_correction:=true \
  heading_yaw_offset_deg:=-1.347 \
  publish_sensor_tf:=false \
  publish_control_state:=true \
  use_quantized_gnss_control_state:=true \
  quantized_control_yaw_tau_sec:=0.0 \
  quantized_control_grid_yaw_deg:=-1.347
```

Terminal 9의 bag tag는 `quantized_full_route_no_actors`로 지정한다. 이 실험은
Planning/TF의 Global state를 바꾸지 않고 controller용 state만 교체한다. 결과가
Smooth production보다 좋아도 문서의 offline/open-loop/음영 안전 gate와 함께 검토한
뒤에만 기본값을 바꾼다.

기본값 대신 과거 closed-loop 후보를 재검증할 때는 위 `localization` 한 줄만 다음 중
하나로 교체한다. 다른 터미널 구성은 동일하게 유지한다.

```text
localization_sigma_0p10_qxy_0p05  # sigma=0.10 m, Global Qxy=0.05
localization_sigma_0p05_qxy_0p05  # sigma=0.05 m, Global Qxy=0.05
```

`enable_traffic_light_planner`의 기본값은 `true`이며 인자를 생략한 production
`planning`은 기존과 동일하다. 이 구성에서도 Sunshine과 RViz는 유지할 수 있다.
Bag은 Camera, LiDAR, PointCloud topic을 포함하지 않는다. 주행 후
`Ego position distance / integral(velocity.x dt)`가 `0.98~1.02`인지 먼저 확인한다.

## E25 복귀용 수동 통합 launch 참고

아래 절차는 E25 복귀용 구 입력의 로컬 재현 계약이다. 현재 E12 production·대회 실행은 위의 alias별 실행과
[배포 안내](../bevfusion/BASE5_BEVFUSION_PRODUCTION_HANDOFF.md)를 따른다. 위 현장
절차와 섞으면 localization, detector, tracking, traffic-light 또는 control publisher가
중복될 수 있으므로 둘 중 하나만 사용한다.

## Terminal 1: ROS master

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
roscore
```

이 터미널은 전체 실행이 끝날 때까지 유지한다.

## Terminal 2: MORAI sensor, Ego, control UDP gateway

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source scripts/workspace/source_aw149.sh

roslaunch sensor_interface morai_udp_interface.launch
```

이 launch 하나가 현재 설정 기준으로 camera/IMU/GPS 수신, Velodyne LiDAR,
`/Ego_topic` 수신과 `/ctrl_cmd` 송신을 실행한다.

## Terminal 3: 입력 topic 점검

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source scripts/workspace/source_aw149.sh

rostopic hz /sensor/lidar1
rostopic hz /sensor/camera/front/compressed
rostopic hz /sensor/camera/front_left/compressed
rostopic hz /sensor/camera/front_right/compressed
rostopic hz /sensor/camera/back/compressed
rostopic hz /Ego_topic
rostopic info /ctrl_cmd
```

각 `rostopic hz`는 확인 후 `Ctrl-C`로 다음 항목을 본다. `/ctrl_cmd`는 controller를
켜기 전 publisher가 없어도 정상이고 UDP sender subscriber가 보여야 한다.

## Terminal 4: BEVFusion Docker inference server

`.env`의 사용자별 절대경로에 의존하지 않고 명령에서 checkpoint를 지정한다.

```bash
cd "$HOME/AjouNice2026"

AJOUNICE_ROOT="$PWD" \
BEVFUSION_CHECKPOINT="$HOME/bevfusion_artifacts/checkpoints/base3_bf0_f40_e25_inference.pth" \
docker compose -f docker/bevfusion/compose.yaml up -d --force-recreate inference

docker compose -f docker/bevfusion/compose.yaml logs -f inference
```

로그에서 `BEVFusion model is ready`와 `Listening`을 확인한다. 로그 follow는
`Ctrl-C`로 빠져나와도 container는 계속 실행된다.

## Terminal 5: inference server healthcheck

```bash
cd "$HOME/AjouNice2026/docker/bevfusion"
PYTHONPATH=. python3 scripts/healthcheck.py
```

정상 출력은 `model_loaded=True`다. `Connection refused`이면 Terminal 4의
container와 port `29500`부터 확인한다.

## Terminal 6: BEVFusion ROS bridge

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source scripts/workspace/source_aw149.sh

roslaunch bevfusion_ros base3_online.launch start_rviz:=false
```

이 launch가 sensor를 Docker에 전달하고 canonical detection topic
`/perception/object_recognition/detection/objects`를 발행한다. 기본 class threshold는
`0.40/0.15/0.10`이다.

## Terminal 7: detector 출력 점검

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source scripts/workspace/source_aw149.sh

rostopic info /perception/object_recognition/detection/objects
rostopic hz /perception/object_recognition/detection/objects
rostopic echo -n 1 /perception/bevfusion/status
rostopic echo -n 1 /perception/bevfusion/latency_ms
```

publisher는 `/bevfusion_ros_bridge` 하나여야 한다. FastBEV나 PointPainting publisher가
동시에 있으면 중복 detector를 종료한다.

## Terminal 8: planning, tracking, Map-CV

### Localization을 별도 터미널에서 먼저 실행

대회 client는 `/Competition_topic + GPS + IMU`를 제공하며 `/Ego_topic`을 production
입력으로 사용하지 않는다. `docs/common_alias.txt`를 source한 뒤 대회용 alias를 실행한다.

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source scripts/workspace/source_aw149.sh
source docs/common_alias.txt

localization initial_position_source:=official_start
```

alias는 `gt_source=competition`, heading correction `-1.347 deg`,
`publish_sensor_tf=false`를 명시한다. 아래 planning 명령에는 반드시
`start_localization:=false`를 추가한다. 다른 localization provider를 동시에 실행하면
안 된다.

MRP에서 같은 estimator를 `/Ego_topic`으로 시험할 때만 `localization_mrp`를
사용한다. 로컬 PC에서 MR을 실행할 때는 `localization`이 맞다.
`localization_morai_gt`와 구형 `maptf_*`는 대회 fallback이 아니다.

### 안전한 첫 확인: controller OFF

먼저 trajectory와 신호등 판단을 확인할 때 사용한다.

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source scripts/workspace/source_aw149.sh

roslaunch ajounice_planning_integration morai_planning_stack.launch \
  start_detector:=false \
  start_controller:=false \
  traffic_light_inference_python:="$HOME/miniconda3/envs/traffic_light_gpu/bin/python" \
  traffic_light_device:=0 \
  maximum_tracking_internal_prediction_age_sec:=0.5
```

### 폐루프 주행: controller ON

MORAI가 `AV-ExternalCtrl`이고 detector, localization, route와 trajectory를 확인한 뒤
위 launch를 종료하고 다음으로 실행한다.

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source scripts/workspace/source_aw149.sh

roslaunch ajounice_planning_integration morai_planning_stack.launch \
  start_detector:=false \
  start_controller:=true \
  traffic_light_inference_python:="$HOME/miniconda3/envs/traffic_light_gpu/bin/python" \
  traffic_light_device:=0 \
  maximum_tracking_internal_prediction_age_sec:=0.5
```

별도 Competition CTRA localization을 실행 중이면 위 두 명령 모두에 다음 한 줄을
추가한다.

```text
start_localization:=false
```

`start_detector:=false`는 perception을 끄는 옵션이 아니다. Terminal 6의 외부
BEVFusion이 이미 canonical detection topic을 발행하므로 planning launch가 다른
detector를 중복 실행하지 않게 하는 옵션이다.

이 통합 launch는 localization, tracker, UNKNOWN stationary policy, Map-CV,
traffic-light recognition, planner를 함께 실행한다. `start_controller:=true`일 때만
PP-PI controller를 추가로 실행한다.

신호등 GPU 환경이 없는 planning 팀 PC에서는 팀의 검증된 `/usr/bin/python3`
환경을 그대로 사용할 수 있으므로 두 `traffic_light_*` override를 생략한다. 이
RTX 5060 Ti PC에서는 `traffic_light_gpu` override가 기준이다.

## Terminal 9: RViz

### MORAI 개발·디버그

분홍색 `/Object_topic` GT box를 함께 본다.

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source scripts/workspace/source_aw149.sh

roslaunch bringup aw149_planning_rviz.launch show_morai_gt:=true
```

### 대회·production

GT가 없다는 것을 명시하고 실행한다.

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source scripts/workspace/source_aw149.sh

roslaunch bringup aw149_planning_rviz.launch show_morai_gt:=false
```

RViz `Route Control` 패널에서 차량이 공식 시작 위치에 정지한 것을 확인하고 Route A를
요청한다. detector box, tracking box/label, Map-CV 1/2/3초 predicted path,
Lanelet map과 planning trajectory를 확인한다.

## Terminal 10: full stack health

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source scripts/workspace/source_aw149.sh

rostopic hz /localization/kinematic_state
rostopic hz /perception/object_recognition/detection/objects
rostopic hz /perception/object_recognition/tracking/objects
rostopic hz /perception/object_recognition/objects
rostopic hz /planning/trajectory
rostopic hz /ctrl_cmd
rostopic echo -n 1 /planning/validation_status
```

각 명령은 하나씩 확인한다. 제어가 안 될 때는 `/ctrl_cmd`부터 억지로 고치지 말고
입력→detection→tracking→prediction→trajectory→controller 순서로 끊긴 지점을 찾는다.

## `/Object_topic`이 없는 실대회

Production detector, tracker, Map-CV와 planning은 `/Object_topic`을 입력으로 사용하지
않는다. 실제 입력은 다음과 같다.

- detector: camera와 LiDAR.
- tracker: canonical detection topic.
- Map-CV: tracked objects와 Lanelet map.
- planning: predicted objects, localization, map, traffic signal.

`/Object_topic`은 offline 정량평가, oracle 실험 및 분홍색 GT RViz overlay에만
사용한다. 따라서 대회에서 topic이 광고되지 않아도 perception→planning→control은
계속 동작한다. ROS subscriber는 publisher가 없다고 process error를 내지 않고 단순히
메시지를 받지 않는다.

대회에서는 `show_morai_gt:=false`를 명시한다. GT가 없는 상태에서 `true`여도 핵심
stack은 멈추지 않지만, 빈 debug display를 production 기능으로 오해할 수 있으므로
권장하지 않는다. RViz의 rosbag-record topic 목록에 `/Object_topic`이 있어도 없는
topic만 기록되지 않을 뿐 다른 topic 기록은 계속된다.

`/Object_topic`과 `/Ego_topic`은 별도 계약이지만 현재 대회 실행은 둘 모두 필수 입력으로
사용하지 않는다. 통합 launch의 GT localization은 기본 OFF이며, 위 별도 localization이
Competition과 공식 START 초기화를 사용한다. 개발용 Ego 초기화·`od_gt`·GT overlay를
대회에 함께 켜지 않는다. 초기화 조건과 남은 LAN 검증은 Localization CURRENT를 따른다.

## 종료 순서

1. controller가 있는 Terminal 8을 `Ctrl-C`로 먼저 종료한다.
2. RViz, BEVFusion ROS bridge, UDP gateway를 종료한다.
3. Docker server를 종료한다.

```bash
cd "$HOME/AjouNice2026"
docker compose -f docker/bevfusion/compose.yaml down
```

4. 마지막으로 roscore를 종료한다.
