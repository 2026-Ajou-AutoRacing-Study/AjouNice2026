# MORAI full stack 터미널별 실행법

## 범위

이 문서는 하나의 shell wrapper가 아니다. 각 구성요소를 별도 터미널에서 실행해
문제가 생긴 계층을 바로 확인할 수 있도록 정리한 운영 runbook이다.

기준 구성은 다음과 같다.

```text
MORAI sensors/Ego
  -> UDP bridge
  -> B3-BF0-F40 E25 BEVFusion
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

현재 production detector는 `B3-BF0-F40 E25`, class threshold는
Vehicle/Pedestrian/Obstacle `0.40/0.15/0.10`이다. E10은 rollback
baseline이며 BF1은 실험 모델로 이 runbook의 기본 모델이 아니다.

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
test -f kcity_map/lanelet_map/lanelet2_map_v84.osm
```

MORAI에서는 다음을 확인한다.

1. v84 map과 원하는 competition scenario를 연다.
2. Sensor Network와 Ego Network를 현재 PC 설정에 맞게 연결한다.
3. 실제 제어 전에 Ego control mode를 `AV-ExternalCtrl`로 둔다.
4. 차량은 Route A 공식 시작 위치에서 정지 상태로 둔다.

## 대회 현장 권장: alias + `tee` 로그 세션

`tee`를 사용하면 화면 출력과 파일 저장이 동시에 이루어진다. 모든 터미널은 같은
로그 세션을 사용해야 하므로 Terminal 1에서 반드시 `field_log_new`를 먼저 실행한다.
그 외 터미널에서 세션이 없거나 다른 로그 경로가 잡히면 `field_log_tee`가 오류를 내고
pipeline을 실패 처리하므로 로그 누락을 바로 확인할 수 있다.

로그 파일명은
`<세션시각>_<구성요소>_<해당명령시작시각>.log` 형식이다. `field_log_tee`는
`tee -a`를 사용하지 않으며 호출할 때마다 새 파일을 만든다. 같은 구성요소를 같은 초에
다시 실행해도 `_01`, `_02`가 붙으므로 기존 로그를 덮어쓰거나 이어붙이지 않는다.

각 터미널의 공통 준비는 다음 네 줄이다. `AJOUNICE_ROOT`를 현재 checkout으로 명시해
이전에 열었던 shell의 환경변수가 섞이지 않게 한다.

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail
```

### Terminal 1: 로그 세션 생성 + ROS master

반드시 가장 먼저 실행하고 전체 실행이 끝날 때까지 유지한다.

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail

field_log_new
rc 2>&1 | field_log_tee roscore
```

`field_log_new`는 `artifacts/field_logs/<YYMMDD_HHMMSS>_session.txt`에 Git SHA,
host, ROS 환경을 기록하고 `/tmp/ajounice_field_log_prefix`를 원자적으로 갱신한다.
같은 초에 세션을 다시 만들어도 기존 로그를 덮어쓰지 않는다.

### Terminal 2: 현장 LAN UDP gateway

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail

udp_lan 2>&1 | field_log_tee udp_lan
```

camera/IMU/GPS 수신, Velodyne LiDAR 변환, `/Ego_topic` 수신과 `/ctrl_cmd` UDP
송신을 담당한다.

### Terminal 3: sensing static TF + 차량 모델

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail

sensing_lan 2>&1 | field_log_tee sensing
```

`udp_lan`에는 sensor static TF가 포함되지 않으므로 생략하면 안 된다. 이 launch의
`enable_ego_tf` 기본값은 `false`라서 Terminal 4의 localization TF와 중복되지 않는다.
`sensing_lan`은 UDP gateway와 같은 competition sensor JSON에서 TF 위치를 읽는다.

### Terminal 4: localization + TF

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail

localization 2>&1 | field_log_tee localization
```

이 alias는 `localization_gnss_imu_gt_ctra.launch`를 실행한다. `sensing_lan`이 sensor
static TF를 담당하므로 localization의 `publish_sensor_tf` 기본값은 `false`로 유지한다.

### Terminal 5: BEVFusion detection

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail

bevfusion 2>&1 | field_log_tee bevfusion
```

이 alias는 Docker inference server와 ROS bridge의 생명주기를 함께 관리한다. canonical
detection topic에 PointPainting/FastBEV/다른 bridge publisher가 있으면 안전하게
거부한다. 다만 Docker container 내부의 상세 stdout 전체는 이 파일에 계속 복제되지
않으므로 필요할 때만 별도 터미널에서 아래 진단 로그를 추가한다.

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail

docker compose -f docker/bevfusion/compose.yaml logs --follow --tail=100 inference \
  2>&1 | field_log_tee bevfusion_inference
```

### Terminal 6: tracking + map-based prediction

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail

tracking 2>&1 | field_log_tee tracking
```

### Terminal 7: traffic-light recognition

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail

traffic 2>&1 | field_log_tee traffic
```

### Terminal 8: planning

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail

planning 2>&1 | field_log_tee planning
```

`planning`은 map, route requester와 planning graph만 실행한다. 이름과 달리 trajectory
follower/control은 포함하지 않는다.

### Terminal 9: 공용 Planning RViz

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail

planviz show_morai_gt:=false 2>&1 | field_log_tee planviz
```

실대회 production에서는 GT overlay를 끈다. MORAI 개발 중 `/Object_topic` 비교가
필요할 때만 `show_morai_gt:=true`로 바꾼다.

### Terminal 10: control — 안전 확인 후에만 실행

처음에는 이 터미널을 실행하지 않고 `/planning/trajectory`와 validation 상태까지
확인한다. 차량 제어권과 비상정지 담당을 확인한 뒤에만 실행한다.

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt
set -o pipefail

traj_fol_a 2>&1 | field_log_tee control
```

`traj_fol_a`만 `/planning/trajectory`를 받아 `/ctrl_cmd`를 만드는 현재 Autoware용
controller다. `traj_fol`이나 다른 `/ctrl_cmd` publisher를 동시에 실행하면 안 된다.

### 로그 확인과 topic-rate snapshot

별도 터미널에서 현재 세션 파일을 확인한다.

```bash
cd "$HOME/AjouNice2026"
export AJOUNICE_ROOT="$PWD"
source docs/common_alias.txt

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

## 수동 통합 launch 참고

아래 절차는 alias별 분리 실행 대신 통합 launch를 사용할 때의 참고 절차다. 위 현장
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

localization
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

주의할 점은 `/Object_topic`과 `/Ego_topic`이 서로 다른 계약이라는 것이다. 현재
MORAI 통합 launch의 localization은 `/Ego_topic`을 사용한다. 대회에서 주최 측이
제공하기로 한 Ego/localization 계약은 별도로 반드시 확인되어야 하며,
`/Object_topic`이 없어도 된다는 결론을 `/Ego_topic`에도 확대하면 안 된다.

## 종료 순서

1. controller가 있는 Terminal 8을 `Ctrl-C`로 먼저 종료한다.
2. RViz, BEVFusion ROS bridge, UDP gateway를 종료한다.
3. Docker server를 종료한다.

```bash
cd "$HOME/AjouNice2026"
docker compose -f docker/bevfusion/compose.yaml down
```

4. 마지막으로 roscore를 종료한다.
