# Sunny 13시 180-scenario 빠른 학습 검증 수집

## 목적

PointPainting 파인튜닝 후 성능 상승 여부를 빠르게 확인하기 위해 날씨는
Sunny, 시간은 13시로 고정하고 서로 다른 180개 scenario를 수집한다.
각 scenario는 `rolling-grouped`로 6개 동기 multisensor frame을 만든다.

- train: 126 scenario, 756 frame
- val: 27 scenario, 162 frame
- test: 27 scenario, 162 frame
- 전체 목표: 180 scenario, 1,080 frame

split은 frame이 아니라 scenario 단위로 고정했다. 한 scenario의 6프레임이
서로 다른 split으로 나뉘지 않는다.

## 생성 결과

- Scenario: `dataset/morai_od/scenarios/sunny13_180_smoke_v2/`
- Metadata: `dataset/morai_od/metadata/sunny13_180_smoke_v2/`
- Capture manifest:
  `dataset/morai_od/manifests/sunny13_180_smoke_v2_sunny_t13.json`
- Split:
  `dataset/morai_od/manifests/sunny13_180_smoke_v2_{train,val,test}.txt`
- 생성 요약:
  `dataset/morai_od/reports/sunny13_180_smoke_v2_generation_summary.json`

v2는 대회용·개인용 공통 DataID만 배치하고, 개인용 런처가 저장한 actor
template를 사용한다. v1의 asset 호환 실패 분석과 상세 정책은
`morai_launcher_asset_compatibility_260807.md`를 참고한다.

MORAI 설치 위치는 다음과 같다.

```text
~/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/
R_KR_PR_K-city_2025/
```

## 실패 허용 정책

runner는 `--continue-on-scene-error`로 실행한다. 개별 scenario에서 충돌,
network 회복 실패 또는 불완전 timestamp 묶음이 발생해도 다음 scenario로
진행한다.

- 실행 중: 장면마다 `ros_capture_poc_report.partial.json` 갱신
- 정상 종료: `ros_capture_poc_report.json`
- 전달 준비 후: `failed_scenarios.jsonl`
- 성공 sample만: `samples.jsonl`

누락 자체는 허용하지만 불완전한 Camera/LiDAR/GT 묶음을 성공 sample로
간주하지 않는다.

## 실행

MORAI에서 Sunny, 13시를 설정하고 Camera 4대·LiDAR Capture Mode와 GT
출력을 켠다. Ego Network와 Simulator Network의 ROS 연결을 확인한다.

```bash
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

RUN_TAG="$(date +%y%m%d_%H%M%S)"
RUN_DIR="/home/luv/morai_cap/sunny13_180_smoke_v2_${RUN_TAG}"
mkdir -p "$RUN_DIR"

set -o pipefail

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest \
  dataset/morai_od/manifests/sunny13_180_smoke_v2_sunny_t13.json \
  --capture-dir "$RUN_DIR" \
  --no-timestamp-prefix \
  --motion-mode rolling-grouped \
  --captures-per-scene 6 \
  --capture-spacing-m 0.6 \
  --speed-kph 3.6 \
  --maximum-motion-m 4.0 \
  --load-settle-sec 0.5 \
  --capture-stable-sec 0.5 \
  --continue-on-scene-error \
  2>&1 | tee "$RUN_DIR/console.log"
```

중간 종료 시 `ros_capture_poc_report.partial.json`을 보존한다. 처음부터
다시 실행하기보다 해당 report를 먼저 확인해 완료 범위를 판단한다.

## 전달 index 생성

정상 종료 후 같은 터미널에서 실행한다.

```bash
python3 dataset/morai_od/scripts/build_sunny13_180_smoke_handoff.py \
  --report "$RUN_DIR/ros_capture_poc_report.json"
```

실행이 중단돼 최종 report가 없다면 partial report로도 현재까지의 전달
index를 만들 수 있다.

```bash
python3 dataset/morai_od/scripts/build_sunny13_180_smoke_handoff.py \
  --report "$RUN_DIR/ros_capture_poc_report.partial.json"
```

학습 담당자에게 `RUN_DIR` 전체를 전달한다. 먼저
`handoff_manifest.json`을 읽고 `samples.jsonl`만 학습 sample index로
사용한다. 실패 원인은 `failed_scenarios.jsonl`, 원본 실행 기록은
`ros_capture_poc_report.json`과 `console.log`에서 확인한다.
