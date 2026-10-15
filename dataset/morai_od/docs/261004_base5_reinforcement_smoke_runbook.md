# Base5 근접·고주로 원거리 보강 — smoke 결과와 런북

상태: EVIDENCE / 본 수집 설계 중. 2026-10-04, `dataset/base5`.

## 이번 단계 결과

- 정지 24/24 scene, 저속 통과 2/2 scene 완료. runtime 실패 0.
- 정지: 근접 12 scene + 고주로 전후방 12 scene, 각 3 frame. train/val 54/18, 총 72.
- 저속 통과: CargoBox/Concrete_Barricade01 각 6 trigger, 12 frame. 같은 차로를 유지하며 옆 객체를 지나간다. 실제 회피 차선 변경이나 planning 폐루프가 아니다.
- scale 1, WoodBox 제외. 차량은 Ioniq/K5, 근접 장애물은 CargoBox/PE_Drum/Concrete_Barricade01이다.
- 고주로는 MGeo의 legal post-taper A432→A427→A444 경로에서 전후방 30/40/60 m를 배치했다. 현재는 같은 차로의 정적 차량이다. 동적·옆차로·합류 다양성까지 완성한 main dataset은 아니다.
- ego 초기속도 0. 통과 요청속도 10 km/h, 실제 trigger 이동량 약 1.05–6.47 m. control service 정상 응답, runner 종료 후 Ego velocity 약 0 확인.
- 원본 정지 약 940.8 MiB, 통과 약 96.2 MiB. 84 selected frame 변환 성공, 미식별 asset 0.
- MR heading -1.347°를 GT 변환 시 한 번 적용, measured geometric BBox catalog 사용. raw cloud, deskew OFF 유지.

## 품질 gate: 성공/실패를 구분한다

- 정지 원본 audit는 **FAIL**: 376 candidate cloud 중 카메라별 2–3개에서 nearest age 100 ms 초과, 최대 113.3 ms. 오류를 숨기거나 tolerance를 늘리지 않았다.
- 변환이 실제 선택한 정지 72 frame의 camera age max 26.93 ms, p95 24.52 ms. 따라서 선택 데이터는 100 ms gate 통과.
- 통과 원본 audit PASS. 선택 12 frame camera age max 25.03 ms, p95 24.42 ms.
- capture-window LiDAR median: 정지 13.21 Hz, 통과 12.10 Hz. 짧은 window counts/duration이므로 장시간 센서 안정성 주장이 아니다.
- info 구조 validator: 정지 72 / 통과 12 frame PASS. sparse pilot만 `--minimum-gt-points 0` 명시, 기존 validator 기본값 5 유지.
- train/val은 scene 단위 분리, test 없음. 소량 반복 layout이므로 val 점수를 일반화 성능으로 제시하지 않는다.
- 변환 중 실행한 최초 `parsed/`는 recorder 종료 전 pass bag 일부가 섞인 중간 산출물이다. **사용 금지**. canonical은 `parsed_static/`, `parsed_pass/` 두 디렉터리다.
- 실제 train pipeline dataloader 54개 로드 성공. 무작위 augmentation/ROI filtering 뒤 빈 GT가 생길 수 있으며 이것을 detector 누락으로 세지 않는다.

## 발견한 입력 문제

대표 1 frame/scene의 corrected GT 내부 LiDAR point:

| Asset | 전방 접근 8m | 측면 3.2m | 통과 후 후방 |
| --- | ---: | ---: | ---: |
| CargoBox | 44 | 0 | 58 |
| PE_Drum | 39 | 0 | 52 |
| Concrete_Barricade01 | 153 | 4 | 214 |
| Ioniq | 342 | 567 | 599 |

측면 CargoBox 이미지를 직접 확인했다. 카메라에 보이는 낮은 객체도 LiDAR 내부점이 0일 수 있다. sparse GT는 raw와 info 진단에 보존하며, 5-point 일괄 제외 정책을 근접 camera-visible addon에 그대로 적용하지 않는다.
단, projection audit는 **가림/차체 masking을 판별하지 못한다**. camera rectangle overlap만으로 모든 zero-point GT를 자동 학습 대상으로 채택하지 않는다.

고주로 Ioniq 전방 30/40/60 m의 대표 point 수는 26/10/6, K5는 23/8/9다.
현재 모델 입력 256×704로 환산한 60m projected rectangle은 약 13×10 px이다.
Pixel estimate uses isotropic resize + bottom crop, superseding the earlier direct H/W normalization estimate.
High-resolution test pipeline: 54/54 frames loaded, 4x3x320x896 images; annotation counts are not post-ROI supervision or accuracy metrics.
320×896 입력은 동일 FOV 기준 픽셀 크기를 늘리지만 실제 인지 개선은 학습/held-out 평가로 확인해야 한다.

거리 요청값은 현재 **asset pivot의 route arc offset**이다. geometric center 보정 후 전방 60m GT는 ±60m ROI 밖에 있을 수 있다. 본 수집은 중심 기준 margin을 두거나 실제 GT 중심 거리로 stratify해야 한다. 확대 ROI 학습 없이 이 GT를 production ±30m에 넣지 않는다.

## 경로

```text
artifacts/datasets/base5_reinforcement_smoke_261004/
  scenarios/manifest.json       # 정지 24 scene
  scenarios/pass_manifest.json  # 저속 통과 2 scene
  runtime.json / pass_runtime.json
  parsed_static/               # canonical 72 frame
  parsed_pass/                 # canonical 12 frame
  raw_static_audit.json / raw_pass_audit.json
  camera_support_static.json / camera_support_pass.json
  gt_audit/                    # 대표 24 frame BEV + 4-camera GT 투영
rosbags/datasets/base5/261004_reinforcement_smoke/ # 정지 24 bag
rosbags/datasets/base5/261004_pass_smoke/          # 통과 2 bag
/home/luv/morai_cap/261004_base5_reinforcement_smoke/
/home/luv/morai_cap/261004_base5_pass_smoke/
```

snapshot event log는 최초 recorder 폴더에 두고, pass 두 bag만 목적별 폴더로 옮겼다. event log의 pass 원래 경로는 이동 전 이력이다. 데이터·bag·생성 scenario는 Git ignore 유지.

## 재현: 이미 MR minimal hybrid 연결된 상태

master/rosbridge/UDP GUI 연결은 [단일 hybrid 런북](morai_mr_mrp_hybrid_scenario_capture_runbook.md)을 따른다. personal overlay를 MR에 추가하지 않는다.
아래 명령은 기존 run을 덮어쓰지 않도록 출력 directory와 RUN_TAG를 새 값으로 바꾼다.

```bash
# 터미널 A — 생성·설치
ajou
/usr/bin/python3 dataset/morai_od/scripts/generate_base5_reinforcement_smoke.py \
  --output artifacts/datasets/base5_reinforcement_smoke_261004/scenarios
/usr/bin/python3 dataset/morai_od/scripts/install_dynamic_capture_set.py \
  artifacts/datasets/base5_reinforcement_smoke_261004/scenarios/manifest.json \
  --network-template configs/network/ego_network_mr_minimal_hybrid.json \
  --install-dir "$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --online-bag-only \
  --runtime-manifest artifacts/datasets/base5_reinforcement_smoke_261004/runtime.json

# 터미널 B — recorder 먼저, ready 출력 확인
ajou
RUN_TAG=base5_reinforcement_retry \
dataset/morai_od/scripts/record_base4_temporal_gated_bag.sh \
  "$PWD/rosbags/datasets/base5/reinforcement_retry"

# 터미널 C — 정지 probe runner
ajou
/usr/bin/python3 -u dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest artifacts/datasets/base5_reinforcement_smoke_261004/runtime.json \
  --capture-dir "$HOME/morai_cap/base5_reinforcement_retry" --no-timestamp-prefix \
  --morai-scenario-dir "$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025" \
  --morai-sensor-data-dir "$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/SensorData" \
  --motion-mode stationary --online-bag-only \
  --load-timeout-sec 10 --scenario-load-attempts 1 --continue-on-scene-error

# 터미널 D — read-only monitor
ajou
/usr/bin/python3 dataset/morai_od/scripts/monitor_ros_capture_progress_v2.py \
  --run-dir "$HOME/morai_cap/base5_reinforcement_retry" \
  --manifest artifacts/datasets/base5_reinforcement_smoke_261004/runtime.json
```

통과 runner는 pass manifest를 별도 runtime으로 설치하고 `--motion-mode rolling-grouped`로 실행한다. 별도 recorder/run directory를 권장한다. Runner 완료 뒤 recorder만 Ctrl-C 종료한다.
이번 monitor 수정은 static manifest의 `capture_policy.captures_per_scene`을 읽어 목표 `72/72`를 표시한다. 이전 `72/24` 표시는 UI 집계 오류였다.

## 다음 gate

1. 이번 결과는 수집/GT/input smoke이며 아직 학습 개선 결과가 아니다.
2. 본 deck에는 여러 일반 도로 anchor, 장애물 yaw·좌우·부분 시야, 실제 고주로 옆차로·동적 속도 다양성을 추가한다. 고주로 한 anchor만 반복하지 않는다.
3. 중심 기준 60m ROI margin과 camera-visible sparse GT 정책을 확정한다. 모든 far label이 train/test filtering 후 남는지 감사한다.
4. Base3 replay를 섞은 짧은 E25-init fine-tuning과 ROI/해상도 독립 A/B를 준비한다. 큰 temporal 실험과 한꺼번에 바꾸지 않는다.
5. 같은 held-out mission bag에서 최초 차량 전달 거리/시간, 근접 검출 유지, ID lifecycle, Detection/Tracking/Map-CV를 함께 평가한다.
