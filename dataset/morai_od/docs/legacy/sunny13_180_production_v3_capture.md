# Sunny 13시 180-scenario production v3 수집

복사·붙여넣기용 명령만 필요한 경우
`docs/sunny13_180_production_v3_commands.md`를 사용한다.

## 목적과 확정 설정

PointPainting의 1차 파인튜닝 성능 상승을 확인하고, 이후 BEVFusion 등
multi-sensor BEV 모델에도 재사용할 180개 독립 장면을 수집한다. MORAI의
날씨는 Sunny, 시간은 13시로 수동 고정한다.

- scenario 180개 × 동기 frame 6개 = 목표 1,080 frame
- train/val/test = 126/27/27 scenario
- 기본 속도 5.4 km/h, 곡선에서 최대 3.6 km/h까지 감속
- 공식 경로 진행거리 기준 1.5 m 간격
- 장면당 최대 진행거리 10.5 m
- 곡률에 따라 lookahead 1.8→1.2 m, 조향 제한 0.35→0.45 rad
- 개인용 MORAI `front_steer` 부호 `-1`
- 한 장면 실패 시 기록하고 다음 장면 계속 진행

## 생성 결과 검증

- Scenario: `dataset/morai_od/scenarios/sunny13_180_production_v3/`
- Metadata: `dataset/morai_od/metadata/sunny13_180_production_v3/`
- Manifest:
  `dataset/morai_od/manifests/sunny13_180_production_v3_sunny_t13.json`
- Split:
  `dataset/morai_od/manifests/sunny13_180_production_v3_{train,val,test}.txt`
- 생성 요약:
  `dataset/morai_od/reports/sunny13_180_production_v3_generation_summary.json`

생성 후 자동 검사 결과는 다음과 같다.

- 180개 anchor가 모두 고유함
- anchor 간 경로 누적거리 최소 3.523 m
- split 126/27/27
- 개인용 런처에서 누락된 `Woman2`, `Girl2` 미사용
- 보행자 중심 간 최소 거리 2.052 m
- 객체 수 범위: NPC 0~8, 보행자 0~8, 장애물 0~4

`camera_seam_occlusion`은 곡선에서 ±45°/±135° 카메라 경계 객체를 만들기
위해 도로 중심선으로부터 최대 7 m의 보도·도로변을 허용한다. MGeo 정적
충돌 검사와 Ego 주행 회랑 회피는 그대로 적용한다.

## MORAI 사전 준비

수집 전 다음 상태를 만든다.

1. 개인용 MORAI에서 `R_KR_PR_K-city_2025` 맵을 연다.
2. Sunny, 13시로 설정한다.
3. Camera 1~4와 LiDAR 5의 Capture Mode 및 필요한 GT 출력을 켠다.
4. Ego Network를 ROS로 Load하고 Connect한다.
5. Simulator Network는 연결하지 않아도 된다. `rolling-grouped`는
   `/ego_setting`이나 Sync Mode를 사용하지 않는다.
6. 아래 순서대로 `roscore`, `rosbridge_websocket`, 수집기를 실행한다.

production sidecar는 개인용 MORAI의 Ego Network 전체를 ROS
(`connectedType=3`)로 유지한다. `/ScenarioLoad`, `/SaveSensorData`,
`/Ego_topic`, `/Object_topic`, `/ctrl_cmd`가 같은 ROS 연결 수명주기를 갖는다.

제어팀 설정은 Ego Network 전체가 UDP(`connectedType=1`)이며 persistent
`morai_udp_interface.launch`가 상태와 제어를 변환한다. 개인용 ROS profile에서
ScenarioLoad handler 하나만 UDP로 바꾸는 hybrid 구성은 실제 런처에서
Ego/Simulator Network 세션이 끊겼으므로 사용하지 않는다.

`rostopic list`는 topic 이름만 보여 주므로 실제 MORAI 연결은
`rostopic info /ScenarioLoad`, `/SaveSensorData`, `/ctrl_cmd`의 subscriber로
확인한다.

## 최초 한 번: 수정된 시나리오 설치

현재 PC에는 이미 설치되어 있다. 새 PC, 새 MORAI 설치 또는 시나리오 재생성
후에는 다음 명령으로 180개 JSON과 Ego Network sidecar를 함께 설치한다.

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_sunny13_180_production_v3_set.py \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
```

출력 마지막에 `MORAI install:` 경로가 나오는지 확인한다.

## 전체 수집 터미널 순서

### 터미널 1: ROS master

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash

roscore
```

### 터미널 2: MORAI rosbridge

터미널 1의 `roscore`가 뜬 뒤 실행한다.

```bash
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

roslaunch rosbridge_server rosbridge_websocket.launch
```

그다음 MORAI Ego Network에서 ROS profile을 Connect한다.

### 터미널 3: 연결 사전 점검

```bash
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

rostopic info /ScenarioLoad
rostopic info /SaveSensorData
rostopic info /ctrl_cmd
rosservice info /Service_MoraiEventCmd
```

세 topic의 `Subscribers`와 service의 `Providers`에
`/rosbridge_websocket`이 있어야 한다. Simulator Network UI가
Disconnected여도 이 네 항목이 정상이면 수집 가능하다.

### 터미널 3: 전체 180개 수집

프로젝트 루트에서 실행한다.

```bash
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

RUN_TAG="$(date +%y%m%d_%H%M%S)"
RUN_DIR="/home/luv/morai_cap/sunny13_180_production_v3_${RUN_TAG}"
mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" | tee /tmp/morai_sunny13_current_run_dir

set -o pipefail

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest \
  dataset/morai_od/manifests/sunny13_180_production_v3_sunny_t13.json \
  --capture-dir "$RUN_DIR" \
  --no-timestamp-prefix \
  --motion-mode rolling-grouped \
  --scenario-load-attempts 1 \
  --continue-on-scene-error \
  2>&1 | tee "$RUN_DIR/console.log"
```

속도·간격·곡선 제어 값은 manifest에서 자동으로 읽으므로 명령행에서 다시
지정하지 않는다. 예상 소요시간은 기존 v3 실측을 기준으로 약 47~55분이다.

production에서는 `--scenario-load-attempts 1`을 사용한다. 객체 한 개 누락처럼
이미 로드된 장면의 품질 문제 때문에 같은 장면을 즉시 다시 load하면 MORAI
network session이 다시 초기화되어 다음 장면까지 막힐 수 있다. 해당 장면은
failed scene으로 기록하고 다음 장면을 새로 load하는 편이 대량 수집에 더
안전하다.

### 터미널 4: 실시간 모니터링

수집 명령이 저장한 정확한 실행 폴더를 읽으므로 과거 진단 폴더를 잘못
선택하지 않는다.

```bash
cd ~/AjouNice2026

RUN_DIR="$(< /tmp/morai_sunny13_current_run_dir)"

python3 dataset/morai_od/scripts/monitor_ros_capture_progress.py \
  --run-dir "$RUN_DIR"
```

터미널을 새로 열었거나 `/tmp` 기록이 없다면 실제 폴더를 직접 지정한다.

```bash
python3 dataset/morai_od/scripts/monitor_ros_capture_progress.py \
  --run-dir /home/luv/morai_cap/<실제_실행_폴더>
```

monitor는 시도/완료/실패 scene 수, 검증 frame 수, 경과시간, 예상 잔여시간,
최근 실패 ID와 마지막 console log를 2초마다 갱신한다. 첫 scene이 끝나기
전에는 partial report가 없으므로 `console.log`의 마지막 줄만 표시한다.

`rolling-grouped`는 `/ctrl_cmd`와 `/Service_MoraiEventCmd`를 사용한다.
runner는 ScenarioLoad 뒤 Ego Network의 `/ctrl_cmd` 재연결을 기다리고
control-mode service를 최대 `network-reconnect-timeout-sec` 동안 기다린다.
Ego Network가 자동 복구되지 않으면 이 대기 시간 안에 UI의 Ego Network를
다시 Connect한다. 연결 전에는 해당 scene의 주행·캡처를 시작하지 않는다.

MORAI가 rosbridge client를 교체하는 짧은 구간에는 이전 연결이 살아 있어
연결 완료로 보이거나 `/Service_MoraiEventCmd`가 `handler returned None`을
반환할 수 있다. runner는 새 Ego 위치와 객체 수로 실제 scene identity를
먼저 확인한다. runner 자체 기본값은 ScenarioLoad 3회 재시도이지만, 이번
production 명령은 network 재초기화를 줄이기 위해 1회만 시도하고 실패를
report에 남긴 뒤 다음 scene으로 진행한다. scene 확인 후 control-mode
service는 최대 3회 재시도한다. ExternalCtrl/D 설정 직후 별도 brake 명령은
보내지 않고 velocity CtrlCmd를 바로 시작한다.

### 중단 후 같은 폴더에서 재개

`Ctrl+C` 또는 ROS 종료가 감지되면 현재 scene을 가짜 실패로 기록하지 않고
partial report를 남긴다. 다음 명령은 기존 성공 scene과 파일을 보존하고,
실패했거나 아직 시도하지 않은 scene만 다시 실행한다.

```bash
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

RUN_DIR=/home/luv/morai_cap/<중단된_실행_폴더>

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest \
  dataset/morai_od/manifests/sunny13_180_production_v3_sunny_t13.json \
  --capture-dir "$RUN_DIR" \
  --no-timestamp-prefix \
  --motion-mode rolling-grouped \
  --scenario-load-attempts 1 \
  --continue-on-scene-error \
  --resume-report "$RUN_DIR/ros_capture_poc_report.partial.json" \
  2>&1 | tee -a "$RUN_DIR/console.log"
```

resume report에서는 `status=completed`인 scene만 건너뛴다. 과거 실패와
미시도 scene은 다시 실행하며, 최종 report에는 보존된 성공 결과와 새
결과를 합친다.

## 실패와 전달

### 대형 버스 링크 제약

`s13p3_005` 초기본은 `DEFAULT_BUS`를 길이 약 25.3 m인 짧은 합류 링크
`A2256W000319`에 배치했다. 같은 장면을 세 번 로드했을 때 다른 9개 객체는
모두 나타났지만 버스만 항상 누락되어 일시적 rosbridge 문제가 아닌 MORAI
actor 초기화 거부로 판정했다. 동일 bus asset은 다른 긴 링크에서 정상
로드되므로 asset 자체는 유효하다.

생성기는 이제 `bus_van` 계열을 길이 35 m 이상이고 양쪽 링크 끝에서 각각
6 m 이상 떨어진 표본에만 배치한다. 이 제약으로 `s13p3_005`의 버스는
`A2256W000323`으로 이동했으며, 180개 중 bus/van 배치가 있던 14개 장면만
재생성했다. 해당 장면들은 실제 캡처 전에 새 버전이 MORAI Scenario 폴더에
설치되어 있어야 한다.

수정본 `s13p3_005` 단일 검증 결과는 다음과 같다.

- NPC 3, 보행자 5, 장애물 2 모두 로드
- `DEFAULT_BUS` 포함 확인
- Camera 1~4와 LiDAR 5 모두 저장
- 필수 GT 누락 0, 총 29개 파일, `verified=true`

최종 report는 `ros_capture_poc_report.json`, 진행 중 report는
`ros_capture_poc_report.partial.json`이다. 일부 장면 실패는 전체 batch를
무효화하지 않는다. 실패 ID와 이유를 보존하고 완료 장면만 학습 index로
만든다.

```bash
python3 dataset/morai_od/scripts/build_sunny13_180_smoke_handoff.py \
  --report "$RUN_DIR/ros_capture_poc_report.json"
```

학습 담당자에게 `RUN_DIR` 전체를 전달한다. `samples.jsonl`은 성공한 동기
frame만, `failed_scenarios.jsonl`은 누락 장면과 원인을 담는다. 동일 scene의
6개 frame은 항상 같은 split에 속한다.

## 2026-08-08 전체 수집 결론

최종 report
`/home/luv/morai_cap/sunny13_180_production_v3_260808_210602/ros_capture_poc_report.json`
기준 결과는 다음과 같다.

- 시도 180, 완료 127, 실패 53, 성공률 70.6%
- 검증 완료 frame 762개; 완료 frame의 Camera 1~4·LiDAR·GT 누락 0
- 48.06분, 약 3.8 GB
- 실패 9개는 `sparse_empty_background`에서 객체가 없기 때문에 생성된
  0바이트 LiDAR `Instance.txt`를 runner가 잘못 거부한 false failure
- 나머지 대부분은 특정 차량 asset이 서로 다른 링크에서도 반복적으로
  로드되지 않은 actor mismatch

반복 누락률이 높아 production v4 자동 풀에서 격리한 차량은 Grandeur,
Porter2, DEFAULT_SUV, DEFAULT_BUS, Niro EV, Xcient Mixer다. 호환성 catalog에서
삭제하지는 않으며 수동 재검증 대상에 남긴다. 상세 수치는
`configs/morai_runtime_asset_reliability_260808.json`에 기록했다.

실패 분석은 다음 명령으로 재현할 수 있다.

```bash
python3 dataset/morai_od/scripts/analyze_morai_capture_failures.py \
  --report \
  /home/luv/morai_cap/sunny13_180_production_v3_260808_210602/ros_capture_poc_report.json \
  --manifest \
  dataset/morai_od/manifests/sunny13_180_production_v3_sunny_t13.json
```
