# Sunny 13:00 v6 600장면 수집 명령

## 터미널 1: 전체 수집

```bash
# 600장면, 최대 3,600프레임을 실패 장면을 건너뛰며 수집
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

RUN_TAG="$(date +%y%m%d_%H%M%S)"
RUN_DIR="/home/luv/morai_cap/sunny13_600_production_v6_${RUN_TAG}"

mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" | tee /tmp/morai_sunny13_600_current_run_dir

set -o pipefail

systemd-inhibit \
  --what=sleep:idle \
  --mode=block \
  --why="MORAI Sunny13 v6 dataset capture" \
  python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest \
  dataset/morai_od/manifests/sunny13_600_production_v6_sunny_t13.json \
  --capture-dir "$RUN_DIR" \
  --no-timestamp-prefix \
  --motion-mode rolling-grouped \
  --scenario-load-attempts 1 \
  --continue-on-scene-error \
  2>&1 | stdbuf -oL awk \
    '/Scenario verified|Rolling-grouped complete|Scene .*failed|auto-capture|summary:|\[WARN\]|\[ERROR\]/ {print; fflush()}' \
  | tee "$RUN_DIR/console.log"
```

## 터미널 2: 실시간 모니터링

```bash
# 완료·실패 장면, 저장 프레임과 예상 잔여시간 표시
cd ~/AjouNice2026

RUN_DIR="$(< /tmp/morai_sunny13_600_current_run_dir)"

python3 dataset/morai_od/scripts/monitor_ros_capture_progress.py \
  --run-dir "$RUN_DIR"
```

## 중단 후 재개

```bash
# 기존 성공 장면은 보존하고 실패·미실행 장면만 같은 폴더에서 재실행
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

RUN_DIR="$(< /tmp/morai_sunny13_600_current_run_dir)"

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest \
  dataset/morai_od/manifests/sunny13_600_production_v6_sunny_t13.json \
  --capture-dir "$RUN_DIR" \
  --no-timestamp-prefix \
  --motion-mode rolling-grouped \
  --scenario-load-attempts 1 \
  --continue-on-scene-error \
  --resume-report "$RUN_DIR/summary.json" \
  2>&1 | stdbuf -oL awk \
    '/Scenario verified|Rolling-grouped complete|Scene .*failed|auto-capture|summary:|\[WARN\]|\[ERROR\]/ {print; fflush()}' \
  | tee -a "$RUN_DIR/console.log"
```

## 완료 결과 확인

```bash
# 최종 완료·실패 수와 프레임 수를 한 번 출력
cd ~/AjouNice2026

RUN_DIR="$(< /tmp/morai_sunny13_600_current_run_dir)"

python3 dataset/morai_od/scripts/monitor_ros_capture_progress.py \
  --run-dir "$RUN_DIR" \
  --once
```
