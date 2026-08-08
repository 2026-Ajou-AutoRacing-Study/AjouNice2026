# Sunny 13시 180-scenario production v3 수집

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

1. 개인용 MORAI에서 `R_KR_PR_K-city_2025` 맵을 연다.
2. Sunny, 13시로 설정한다.
3. Camera 1~4와 LiDAR 5의 Capture Mode 및 필요한 GT 출력을 켠다.
4. Ego Network를 ROS로 연결한다. Simulator Network는 teleport·Sync Mode를
   쓰지 않는 `rolling-grouped` 수집에는 연결하지 않아도 된다.
5. `roscore`, `rosbridge_websocket`이 실행 중인지 확인한다.

현재 생성·설치된 scenario sidecar에서 `/ScenarioLoad` handler의
`commType`은 `3`(ROS)이다. 따라서 현재 production 절차는 ROS transport를
사용한다. runner가 UDP 패킷을 보내는 것만으로는 현재 설정의 MORAI가 이를
수신하지 않는다.

MORAI network JSON은 handler별로 `commType`을 저장하므로 이론적으로
ScenarioLoad만 UDP이고 나머지는 ROS인 hybrid 구성이 가능하다. 실제 제어팀
`configs/network/ego_network_1.json`도 ScenarioLoad handler를
`commType=1`로 둔다. 하지만 production sidecar를 hybrid로 변환하고 개인용
launcher에서 검증하기 전까지 `--scenario-load-transport udp`는 실험용이다.

ScenarioLoad 뒤 Simulator Network UI가 끊기면 정상으로 단정하지 않는다.
`rostopic list`는 topic 이름만 보여 주므로 실제 MORAI 연결은
`rostopic info /ScenarioLoad`, `/SaveSensorData`, `/ctrl_cmd`의 subscriber로
확인한다.

## 전체 수집 명령

프로젝트 루트에서 실행한다.

```bash
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

RUN_TAG="$(date +%y%m%d_%H%M%S)"
RUN_DIR="/home/luv/morai_cap/sunny13_180_production_v3_${RUN_TAG}"
mkdir -p "$RUN_DIR"

set -o pipefail

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest \
  dataset/morai_od/manifests/sunny13_180_production_v3_sunny_t13.json \
  --capture-dir "$RUN_DIR" \
  --no-timestamp-prefix \
  --motion-mode rolling-grouped \
  --continue-on-scene-error \
  2>&1 | tee "$RUN_DIR/console.log"
```

속도·간격·곡선 제어 값은 manifest에서 자동으로 읽으므로 명령행에서 다시
지정하지 않는다. 예상 소요시간은 기존 v3 실측을 기준으로 약 47~55분이다.

100개 이상의 장기 수집은 별도 터미널에서 monitor를 실행한다. `--run-dir`
없이 실행하면 `/home/luv/morai_cap`에서 가장 최근에 갱신된 실행 폴더를
자동 선택한다. 따라서 이후 다른 100개 이상 batch에도 그대로 재사용한다.

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/monitor_ros_capture_progress.py
```

특정 실행 폴더를 직접 지정할 수도 있다.

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
먼저 확인하며, 이전 scene이 남아 있거나 actor가 누락되면 ScenarioLoad를
기본 3회 재시도한다. scene 확인 후 control-mode service도 최대 3회
재시도하고 브레이크를 적용한 뒤 수집을 시작한다.

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
  --continue-on-scene-error \
  --resume-report "$RUN_DIR/ros_capture_poc_report.partial.json" \
  2>&1 | tee -a "$RUN_DIR/console.log"
```

resume report에서는 `status=completed`인 scene만 건너뛴다. 과거 실패와
미시도 scene은 다시 실행하며, 최종 report에는 보존된 성공 결과와 새
결과를 합친다.

## 실패와 전달

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
