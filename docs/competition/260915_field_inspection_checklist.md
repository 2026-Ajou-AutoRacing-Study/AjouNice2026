# 2026-09-15 대회 클라이언트 PC 현장점검 체크리스트

## 목적과 원칙

팀 PC를 실제 대회용 MORAI client PC에 연결해 network, sensor, localization,
perception, planning, control의 **실제 입출력 계약**을 확인한다. 현장에서 성능을
감으로 평가하거나 파라미터를 즉석에서 튜닝하는 것이 목적이 아니다.

원칙은 다음과 같다.

1. controller OFF 상태에서 입력부터 trajectory까지 먼저 통과시킨다.
2. 각 단계의 성공 기준과 증거를 기록한 뒤 다음 단계로 넘어간다.
3. 문제가 나면 한 번에 한 변수만 바꾼다.
4. 현장에서 새 daemon이나 알고리즘을 만들지 않는다.
5. 변경 전 명령, 변경값, 시각과 결과를 issue log에 남긴다.
6. `/Object_topic` 같은 GT debug 입력 없이 production graph가 동작하는지 반드시
   별도로 확인한다.

터미널별 기본 실행법은
[`morai_full_stack_by_terminal.md`](../runbooks/morai_full_stack_by_terminal.md)를
사용한다.

## P0: 출발 전에 반드시 준비

### Software와 artifact 동결

- [ ] 사용할 Git branch와 commit SHA를 기록했다.
- [ ] 작업트리에 의도하지 않은 수정이 없다. 수정이 있다면 diff를 별도로 보관했다.
- [ ] `planning_ws/install_isolated`와 `base_ws/install_aw149`가 현재 source에서
      빌드된 결과인지 확인했다.
- [ ] v62 OSM, projector, route config의 파일명과 SHA-256을 기록했다.
- [ ] BEVFusion production weight는 `B3-BF0-F40 E25`이며 SHA-256
      `36816cf769c87d49483d002111abfad8e1afd043afd74a0b9fc54aac82133011`을 확인했다.
- [ ] Docker image가 인터넷 없이 기동하고 healthcheck를 통과한다.
- [ ] traffic-light weight와 실행 Python이 인터넷 없이 import 및 CUDA load된다.
- [ ] launch에 필요한 모든 checkpoint가 로컬에 있다.
- [ ] 최소 100 GB 이상의 기록 여유 공간이 있다.
- [ ] 전원 어댑터, LAN/USB-LAN, 여분 케이블과 멀티탭을 준비했다.

### 현장 rollback 준비

- [ ] production 명령과 실험 명령을 섞지 않는다.
- [ ] heading correction ON/OFF 명령을 둘 다 준비했다.
- [ ] controller OFF launch를 가장 먼저 실행할 수 있다.
- [ ] 모든 ROS 및 Docker process를 종료하는 순서를 알고 있다.
- [ ] 검증된 이전 commit SHA와 weight SHA를 기록했다.
- [ ] 현장에서 Git pull, package 설치, Docker rebuild가 필요하지 않게 준비했다.

## P0: 실제 client topic 계약 확인

가장 먼저 운영 측에 다음을 확인하고 실제 `rostopic list/info/hz`로 대조한다.

| 입력 | 확인할 내용 | 없을 때 영향 |
|---|---|---|
| camera 4개 | topic, message type, 해상도, JPEG 여부, rate | BEVFusion 불가 |
| LiDAR | topic, PointCloud2 fields, frame, rate, point time | BEVFusion 불가 |
| GPS/IMU | topic, frame, unit, timestamp | GNSS+IMU localization 불가 |
| `/Competition_topic` | type, heading 단위·북 기준, velocity/acceleration | 대회 CTRA 계약 확인 필요 |
| `/Ego_topic` | 제공 여부와 공식 사용 허용 여부 | 현재 통합 기본 localization에 영향 |
| `/Object_topic` | 제공 여부 | production에는 영향 없음 |
| control | destination IP/port, mode, gear enum, watchdog | 폐루프 제어 불가 |
| clock | wall time인지 simulation time인지, pause/reset 가능성 | freshness 판단에 영향 |

### 반드시 해결해야 하는 localization 분기

현재 `morai_planning_stack.launch`의 기본 `start_localization=true`는
`localization_morai_gt.launch`, 즉 `/Ego_topic`을 사용한다. 대회용으로 별도 준비한
provider는 `/Competition_topic + GPS + IMU` 기반
`localization_gnss_imu_gt_ctra.launch`다.

따라서 현장에서는 다음 중 실제 공식 계약 하나를 명시적으로 선택한다.

- `/Ego_topic`이 공식 제공되고 사용 허용: 통합 기본 localization을 사용한다.
- `/Competition_topic`이 공식 입력: 통합 launch는 `start_localization:=false`로
  실행하고 CTRA localization을 별도 터미널에서 정확히 한 개만 실행한다.

두 localization을 동시에 켜서 같은 TF와 `/localization/kinematic_state`를 중복
발행하면 안 된다.

## 1. Network와 sensing

### 연결

- [ ] client PC와 팀 PC의 IP, subnet, gateway를 기록했다.
- [ ] 방화벽과 VPN 때문에 UDP가 다른 interface로 빠지지 않는다.
- [ ] 예상 UDP receive port가 실제로 열려 있고 packet count가 증가한다.
- [ ] Sensor Network, Ego Network, control destination이 올바른 PC를 가리킨다.
- [ ] 연결을 끊었다 다시 연결했을 때 process 재시작 없이 복구되는지 확인한다.

### Rate와 timestamp

repository 설정상 기대치는 camera 4개 각각 30 Hz, LiDAR 15 Hz, Ego network
50 Hz다. 이것은 합격값이 아니라 **비교 기준**이며 실제 client 값을 기록한다.

- [ ] 각 camera의 평균/p05 rate와 장시간 max gap을 기록했다.
- [ ] LiDAR 평균/p05 rate와 point count 분포를 기록했다.
- [ ] GPS, IMU, Competition/Ego rate를 기록했다.
- [ ] header stamp가 증가하며 wall time/ROS time과 비정상적으로 벌어지지 않는다.
- [ ] 60초 이상 관찰했을 때 topic이 수초간 멎거나 burst로 몰리지 않는다.
- [ ] MORAI pause/resume 또는 scenario reload 후 timestamp가 역행하는지 확인한다.

### 부하와 지연

- [ ] 센서만 켠 상태의 CPU, RAM, network, GPU baseline을 기록했다.
- [ ] detector를 켠 뒤 GPU memory, utilization, temperature를 기록했다.
- [ ] `/perception/bevfusion/latency_ms`의 안정 구간 p50/p95를 기록했다.
- [ ] `/perception/bevfusion/status`의 drop counter가 지속 증가하지 않는다.
- [ ] RViz ON/OFF가 sensor rate와 detector coverage에 미치는 영향을 비교한다.
- [ ] 10분 이상 실행했을 때 지연이 누적되지 않는다.

판정은 단순히 “렉이 보인다/안 보인다”가 아니라 rate, max gap, latency와 drop으로
남긴다.

## 2. Perception, tracking, Map-CV

- [ ] `/perception/object_recognition/detection/objects` publisher가 정확히 하나다.
- [ ] Vehicle/Pedestrian/Obstacle threshold가 `0.40/0.15/0.10`이다.
- [ ] 차량, 보행자, 장애물을 가까운 거리와 20--30 m에서 각각 확인한다.
- [ ] 중앙분리대, 가드레일, 표지판 기둥 등 hard negative 오검출을 기록한다.
- [ ] tracker ID가 정지·가림·재등장 시 얼마나 유지되는지 확인한다.
- [ ] Obstacle/UNKNOWN에 가짜 속도와 이동 predicted path가 생기지 않는다.
- [ ] Vehicle 경로가 Lanelet centerline과 합리적으로 정합한다.
- [ ] 로터리·합류부에서는 후보 path 방향이 차량 진행 방향과 맞는다.
- [ ] detector 중단 후 stale object가 무기한 유지되지 않고 안전하게 소멸한다.

RViz는 simulator debug에서는 GT를 켤 수 있지만, production 확인에서는
`show_morai_gt:=false`로 다시 실행해 GT 없이 동일하게 동작하는지 확인한다.

## 3. 맵과 localization

### 맵 동일성

- [ ] client map/scene 이름과 version을 촬영하고 기록한다.
- [ ] 시작 위치, 도로 형상, lane 수, 합류부, 로터리와 GPS 음영 구간을 샘플과
      대조한다.
- [ ] v62에서 수정된 B, D-1, D-2 stop line이 실제 simulator 신호등 시야와 맞는다.
- [ ] binary planning map과 RViz marker map의 source OSM이 같다.
- [ ] map/TF publisher가 중복되지 않는다.

### 위치·방향 판정

- [ ] 시작점에서 차량 footprint가 lane 중심 및 진행 방향과 맞는다.
- [ ] 직선 20--30 m 주행 시 차량 yaw가 lane tangent와 일정하게 맞는다.
- [ ] 회전·로터리에서 TF가 튀거나 180도 반전하지 않는다.
- [ ] `/localization/kinematic_state`가 연속 발행되고 stamp가 최신이다.
- [ ] GNSS/IMU/Competition input을 일시 차단했을 때 안전 정지와 복구를 확인한다.
- [ ] Z 차이는 planning geometry가 아니라 RViz-only flatten 결과인지 구분한다.

### Heading correction A/B

대회용 CTRA provider를 사용할 때 먼저 현재 기준인 보정 ON을 시험한다.

```bash
roslaunch bringup localization_gnss_imu_gt_ctra.launch \
  gt_source:=competition \
  use_heading_correction:=true \
  heading_yaw_offset_deg:=-1.347 \
  publish_sensor_tf:=false
```

실제 client에서 raw heading이 이미 수정되어 방향이 틀어지면 **동시에 켜지 않고**
위 process를 종료한 뒤 보정 OFF로 A/B한다.

```bash
roslaunch bringup localization_gnss_imu_gt_ctra.launch \
  gt_source:=competition \
  use_heading_correction:=false \
  publish_sensor_tf:=false
```

선택 기준은 눈대중 한 frame이 아니라 시작점, 직선, 회전 구간에서 lane tangent 대비
yaw 오차와 TF 연속성이다. 결과가 같지 않으면 어느 쪽을 선택했는지 rosbag과
issue log에 반드시 남긴다.

## 4. 출발, 판단과 재배치

### 출발 전 정지

- [ ] Route A 요청 전 차량이 움직이지 않는다.
- [ ] controller OFF에서는 `/ctrl_cmd`가 주행 명령을 만들지 않는다.
- [ ] controller ON 후 trajectory가 없거나 invalid일 때 full-brake 또는 안전 정지한다.
- [ ] 시작 시 실제 gear가 P인지 D인지 UI와 Ego status 양쪽에서 기록한다.
- [ ] 현재 UDP sender의 고정 `gear=4`가 client protocol에서 어떤 상태인지 운영 측에
      확인한다. 숫자만 보고 D라고 추측하지 않는다.
- [ ] Route A 요청 후 validation이 정상일 때만 출발한다.
- [ ] autonomous/control-enabled 상태가 실제 제어권과 일치한다.

권장 목표 상태는 “경로 승인 전 정지, 승인 후 명시적으로 출발 가능”이다. P와 D 중
무엇이 정답인지는 client control protocol과 대회 규정으로 확정한다.

### 신호등과 주요 판단

- [ ] A, B, C, D-1, D-2 각각에서 대상 group ID가 맞다.
- [ ] 무관한 신호등을 보고 정지하지 않는다.
- [ ] red에서 stop line 이전에 정지한다.
- [ ] 정지 위치에서 대상 신호등이 camera 안에 보인다.
- [ ] green 전환 후 timeout 없이 출발한다.
- [ ] camera에서 신호가 잠시 사라질 때 stale red/green을 무기한 유지하지 않는다.
- [ ] 정적 장애물에서 회피 또는 안전 정지 결과가 기대와 맞는다.

### 사고 후 checkpoint 순간이동

순간이동은 정상 주행의 연장이 아니라 localization/route/tracker discontinuity fault다.
controller가 켜진 채 바로 시험하지 않는다.

1. [ ] controller OFF 또는 MORAI 안전 정지 상태로 만든다.
2. [ ] 순간이동 직전과 직후의 시각을 기록한다.
3. [ ] pose, TF, route, tracked object와 trajectory가 과거 상태를 유지하는지 본다.
4. [ ] stale trajectory를 controller가 거부하는지 확인한다.
5. [ ] 새 위치가 현재 route에 포함되는지와 재경로 절차를 확인한다.
6. [ ] 자동 복구가 검증되지 않았다면 stack reset + 명시적 route 재요청을 현장 표준으로
       정한다.
7. [ ] 위 확인 후에만 controller ON으로 복구 주행한다.

“순간이동 후 그냥 다시 간다”를 기대값으로 두지 않는다. 안전 정지 후 명시적 재초기화가
필요할 가능성을 기본으로 둔다.

## 5. Control

- [ ] `/planning/trajectory`, localization, control command rate를 함께 기록한다.
- [ ] straight lane에서 lateral oscillation, overshoot와 steering saturation을 본다.
- [ ] 곡선, 로터리, 합류부에서 차선을 밟거나 경계를 넘어가는 시각을 기록한다.
- [ ] target speed 대비 actual speed, 정지거리와 재출발 delay를 기록한다.
- [ ] trajectory 또는 localization loss 시 throttle이 남지 않고 안전 정지한다.
- [ ] control UDP를 끊었다 복구했을 때 갑작스러운 이전 명령이 재사용되지 않는다.
- [ ] emergency stop 방법과 담당자를 주행 전에 합의한다.

### 차선 밟기 판정 프로그램에서 확인할 질문

현재 체크리스트의 “판정 프로그램”은 요구사항이 부족하다. 현장에서 다음을 받아야
재현 가능한 검증이 된다.

- [ ] 판정이 simulator 내부 ground truth인지, camera/차량 footprint 기반인지.
- [ ] 차선의 실선/점선/도로경계 중 무엇을 위반으로 보는지.
- [ ] 타이어 접촉, 차량 footprint 교차, 중심점 이탈 중 어떤 조건인지.
- [ ] 허용 margin, 최소 지속시간과 sampling rate.
- [ ] 차선 변경 허용 구간과 예외 규칙.
- [ ] 순간이동 직후 frame도 위반에 포함되는지.
- [ ] 출력 topic/file, timestamp와 penalty 형식.
- [ ] 우리 rosbag만으로 사후 재현 가능한지.

가능하다면 의도적으로 경계를 한 번 밟는 저속 시험으로 판정 발생 시각과 RViz 위치를
대조한다. 정상 주행 기록과 섞이지 않게 별도 run으로 남긴다.

## 6. 모든 로그 저장

### Primary: rosbag

- [ ] `rosbag record -a`를 쓰지 않고 사전에 정한 production 필수 topic 목록을 쓴다.
- [ ] 주행마다 새 run ID를 사용한다.
- [ ] raw sensor, localization, perception, tracking, prediction, traffic signal,
      planning, validation, control, TF, status와 `/rosout`을 기록한다.
- [ ] `/Competition_topic`, `/Ego_topic`, `/Object_topic`은 존재하는 것만 함께 기록한다.
- [ ] LZ4와 split recording을 사용해 한 파일 손상이 전체 주행을 잃게 하지 않는다.
- [ ] 녹화 시작 후 topic count와 파일 크기 증가를 확인하고 출발한다.
- [ ] 주행 종료 후 정상적으로 Ctrl-C하고 `rosbag info`를 즉시 확인한다.
- [ ] 시작/종료 wall-clock과 scenario/run ID를 별도 메모한다.

Raw 4-camera+LiDAR 기록은 과거 bag 기준 대략 분당 수백 MB 수준이므로, 실제 client
rate에서 5분 시험을 먼저 기록해 분당 용량을 다시 계산한다. 최소 100 GB 여유를
권장한다.

본 주행 전에 동일한 정지 장면을 60초씩 `기록 OFF`와 `기록 ON`으로 비교한다. 센서
rate, max gap, detector latency/drop이 눈에 띄게 나빠지면 full profile을 고집하지
않고 raw sensor 중심 기록과 downstream/status 중심 기록을 서로 다른 짧은 run으로
나눈다. 기록 때문에 production pipeline이 느려지는 구성을 합격으로 간주하면 안 된다.

### Topic Hz 일괄 snapshot

센서·측위·인지·신호등·planning·control의 streaming topic을 같은 20초 구간에서
측정하려면 다음을 실행한다.

```bash
cd "$HOME/AjouNice2026"
bash scripts/runtime/record_field_topic_rates.sh
```

측정 시간을 바꾸거나 결과 루트를 지정할 수도 있다.

```bash
bash scripts/runtime/record_field_topic_rates.sh 30 \
  "$HOME/field_inspection/topic_hz"
```

기본 결과는 `artifacts/field_checks/topic_hz/<YYMMDD_HHMMSS>/`에 저장된다.
`summary.tsv`는 각 topic을 `OK`, `NO_PUBLISHER`, `NO_MESSAGES`로 구분하고 마지막
평균 Hz를 기록한다. `rates/`와 `info/`에는 `/`를 `__`로 치환한 topic 이름별 원본
측정 로그와 publisher/subscriber 정보가 남는다. 실행 당시 Git SHA, NIC 주소,
ROS node/topic 목록도 같은 폴더에 보존한다.

### Secondary: rosbag 실패 대비

새로운 full sensor recorder를 현장에서 즉석 구현하는 것은 피한다. 같은 ROS I/O와
disk를 쓰는 두 번째 recorder는 primary rosbag 장애와 부하를 그대로 공유할 수 있다.

보조 기록은 목적을 줄인다.

- terminal stdout/stderr와 `/rosout` 보존.
- 1초 주기의 핵심 topic rate, latest stamp/age, node 생존 상태.
- detector latency/drop, GPU/RAM/disk/network 사용량.
- localization pose/yaw, route state, validation, control command 요약.
- operator event marker: 출발, 정지, 신호 구간, 사고, 순간이동, reset.

별도 backend daemon을 만들 경우 내일 출발 전에 CPU/RAM/disk 부하, 강제 종료 후 파일
복구, ROS master 재시작을 smoke test한 것만 사용한다. 준비가 안 되면 검증된 terminal
logging과 짧은 분할 rosbag을 우선한다.

## 7. 권장 현장 순서

| 순서 | 시험 | Controller | 다음 단계 조건 |
|---:|---|---|---|
| 1 | client/map/topic 계약 촬영·기록 | OFF | 입력 목록과 버전 확정 |
| 2 | UDP sensor 5분 안정성 | OFF | rate, gap, stamp 정상 |
| 3 | localization correction ON/OFF | OFF | 한 provider만 선택 |
| 4 | detector + tracker + Map-CV | OFF | drop/latency와 출력 정상 |
| 5 | planning output-only + Route A | OFF | trajectory/validation 정상 |
| 6 | 저속 직선·정지·조향 | ON | 안전 정지와 lane 유지 |
| 7 | 신호 A/B/C/D-1/D-2 | ON | 정지선·재출발 정상 |
| 8 | obstacle/로터리/합류 | ON | 판단·경로·제어 정상 |
| 9 | 순간이동 복구 | 최초 OFF | reset/re-route 절차 확정 |
| 10 | GT topic 없는 production 재시험 | ON | full stack 동일 작동 |

각 주행 직전에는 run ID, scenario, Git SHA, map version, localization 선택, heading
correction, detector weight/threshold, controller ON/OFF를 한 줄로 읽어 녹화 영상과
issue log 양쪽에 남긴다. 그래야 서로 다른 설정의 rosbag을 나중에 섞지 않는다.

## 8. Go/No-Go 판정

### No-Go

- sensor topic이 반복적으로 끊기거나 timestamp가 역행한다.
- localization/TF publisher가 중복되거나 map과 yaw가 지속적으로 어긋난다.
- detector가 입력 rate를 감당하지 못하고 지연이 계속 누적된다.
- invalid/stale trajectory에서 control throttle이 남는다.
- red stop 또는 green restart가 재현되지 않는다.
- rosbag과 보조 로그가 모두 남지 않는다.
- emergency stop 담당과 방법이 확정되지 않았다.

### 조건부 Go

- visualization-only Z offset처럼 planning geometry에 영향 없는 문제.
- `/Object_topic` 부재로 GT overlay만 비어 있는 경우.
- 일부 non-critical debug topic 부재지만 production 입력과 로그가 남는 경우.

### Go 증거

- 최소 한 번의 controller OFF end-to-end trajectory 검증.
- 최소 한 번의 저속 폐루프 출발·정지.
- sensor/localization/detection/tracking/prediction/planning/control rate 기록.
- 신호등 구간별 결과와 map/client version.
- 정상 종료된 rosbag과 issue log.

## 9. 현장 issue log 양식

| 시각 | Run ID | 구간/상황 | 현상 | 직전 변경 | 관련 topic/node | 조치 | 결과 |
|---|---|---|---|---|---|---|---|
|  |  |  |  |  |  |  |  |

증상이 발생하면 screenshot/video만 남기지 말고 위 시각을 rosbag time과 연결한다.
