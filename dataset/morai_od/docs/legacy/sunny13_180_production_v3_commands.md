# Sunny 13시 180-scenario 수집 명령어

## 최초 한 번: 시나리오 설치

```bash
# 생성된 시나리오 180개와 Ego Network sidecar를 개인용 MORAI에 설치
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_sunny13_180_production_v3_set.py \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
```

## 터미널 1: ROS master

```bash
# ROS master 실행
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash

roscore
```

## 터미널 2: rosbridge

```bash
# 개인용 MORAI와 ROS를 연결하는 rosbridge 실행
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

roslaunch rosbridge_server rosbridge_websocket.launch
```

## 터미널 3: 연결 확인

```bash
# ScenarioLoad·센서 저장·제어 topic과 control service 연결 확인
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

rostopic info /ScenarioLoad
rostopic info /SaveSensorData
rostopic info /ctrl_cmd
rosservice info /Service_MoraiEventCmd
```

## 터미널 3: 전체 수집

```bash
# Sunny 13시 시나리오 180개를 rolling-grouped 방식으로 수집
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

## 터미널 4: 실시간 모니터링

```bash
# 현재 수집의 완료·실패 장면 수, 프레임 수와 예상 잔여시간 표시
cd ~/AjouNice2026

RUN_DIR="$(< /tmp/morai_sunny13_current_run_dir)"

python3 dataset/morai_od/scripts/monitor_ros_capture_progress.py \
  --run-dir "$RUN_DIR"
```

## 중단 후 재개

```bash
# 기존 성공 장면은 건너뛰고 실패·미실행 장면부터 같은 폴더에서 재개
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

RUN_DIR="$(< /tmp/morai_sunny13_current_run_dir)"

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

## 완료 후 결과 확인

```bash
# 최종 report를 한 번 출력하여 완료·실패 수 확인
cd ~/AjouNice2026

RUN_DIR="$(< /tmp/morai_sunny13_current_run_dir)"

python3 dataset/morai_od/scripts/monitor_ros_capture_progress.py \
  --run-dir "$RUN_DIR" \
  --once
```

## 완료 후 전달 파일 생성

```bash
# 성공 프레임과 실패 시나리오 목록을 학습 전달용 파일로 생성
cd ~/AjouNice2026

RUN_DIR="$(< /tmp/morai_sunny13_current_run_dir)"

python3 dataset/morai_od/scripts/build_sunny13_180_smoke_handoff.py \
  --report "$RUN_DIR/ros_capture_poc_report.json"
```
