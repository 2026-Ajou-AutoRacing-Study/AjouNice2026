# Base3 obstacle benchmark rosbag 평가 (260912)

## 목적

- 기존 `obstacle3` stress bag의 스쿠터 탑승자·비대회형 asset 혼입 문제를 피한다.
- Base3에서 실제 positive로 유지한 정적 장애물 6종만 배치한 사람이 직접 주행한
  rosbag으로 single-frame PointPainting을 비교한다.
- 전체 GT 평가와 LiDAR visibility 평가를 분리한다.

## 입력

- 원본:
  `rosbags/raw_sensor_w_cam/perception_benchmark/260912_195556_bench_static_obstacle_base3_assets_raw.bag`
- 평가 입력:
  `artifacts/evaluation/pointpainting/260912_base3_obstacle_benchmark/input_trim6s.bag`
  - 시작의 정지 구간 6초만 제외했다.
  - 원본은 변경하지 않았다.
- 시나리오:
  `configs/scenario/perception_benchmark/bench_static_obstacle_base3_assets_raw.json`
- 평가 범위: Ego 30 m 이내.
- production operating point: Obstacle score `0.10`.
- LiDAR observable 기준: 유효한 3D 크기와 box 내부 point `5개 이상`.

대상 asset은 `CargoBox`, `PE_Drum`, `TrafficBarrel`, `concrete-barrier`,
`Concrete_Barricade01`, `PE_Firewall_Orange`이다.

## Rosbag 품질

- 원본 길이 `32.08초`, LiDAR `13.06 Hz`, 카메라별 `26.69 Hz`이다.
- Ego 주행은 약 `155.9 m`이며 위치 jump와 collision은 없다.
- 여섯 obstacle은 전 구간에서 같은 ID와 속도 0을 유지한다.
- `/Object_topic`의 `concrete-barrier` 크기는 MORAI가 `(0, 0, 0)`으로 보낸다.
  - center-distance 평가는 가능하지만 mASE와 LiDAR box point count는 유효하지 않다.
  - 이 asset은 visibility slice에서 제외한다.
- 평가 frame의 obstacle GT는 대략 다음과 같다.
  - 유효 box + LiDAR 5점 이상: `309~310`.
  - 유효 box + LiDAR 1~4점: `98~101`.
  - 유효 box + LiDAR 0점: `197~200`.
  - 크기 0인 `concrete-barrier`: `139`.

## Production threshold 0.10 결과

아래 mAP는 center distance `0.5/1/2/4 m` AP 평균이다. PP1의 config-level
proposal floor도 0.10인 원래 배포 조건이다.

| 모델 | Center mAP | Precision / Recall / F1 @1m | mATE @2m | 판정 |
|---|---:|---:|---:|---|
| B3S-PP0 E39 | **0.106** | **0.649 / 0.207 / 0.314** | **0.676 m** | 현재 bag의 가장 안정적인 정밀도와 F1 |
| B3S-PP1 E34 | 0.000 | 0.444 / 0.038 / 0.069 | 0.795 m | GT sampling 이후 obstacle confidence가 지나치게 낮음 |
| S1g E37 | 0.043 | 0.061 / **0.281** / 0.100 | 0.774 m | recall은 높지만 배경 FP가 매우 많음 |

유효 box만 남기고 제외 GT 주변 prediction도 평가에서 무시한 F1은 PP0
`0.179`, PP1 `0.037`, S1g `0.093`이다. LiDAR 5점 이상 slice의 F1은 각각
`0.197`, `0.062`, `0.075`이다.

## Score calibration 진단

- OpenPCDet 내부 proposal floor를 진단 전용으로 `0.01`까지 낮췄다.
- PP1의 최적 obstacle threshold는 약 `0.020`이다.
  - 전체 GT: P/R/F1 `0.220 / 0.150 / 0.178`.
  - 유효 box: P/R/F1 `0.150 / 0.118 / 0.133`.
  - LiDAR 5점 이상: P/R/F1 `0.102 / 0.150 / 0.121`.
- threshold를 낮추면 PP1이 일부 회복되지만 PP0의 score 0.10 F1 `0.314`보다
  낮다. PP1 문제는 calibration만으로 설명되지 않는다.
- PP0의 진단 최적 threshold는 약 `0.053`, 전체 F1 `0.362`이다.
- S1g는 약 `0.217`에서 전체 F1 `0.243`, LiDAR 5점 이상 F1 `0.259`이다.
  다만 310 frame에서 obstacle FP가 `341`개라 정지 장애물로 planning에 넣기에는
  여전히 공격적이다.
- S1g를 proposal floor 0.01로 내리면 frame coverage가 `90.9% -> 46.5%`로
  붕괴하고 58,419개 prediction이 발생한다. 이 설정은 production 후보가 아니다.

## Asset별 관찰

LiDAR 5점 이상 GT의 recall@1m이다. `concrete-barrier`는 크기 0이라 제외한다.

| Asset | PP0 @0.10 | PP1 @0.02 | S1g @0.217 |
|---|---:|---:|---:|
| CargoBox | **0.167** | 0.000 | 0.071 |
| PE_Drum | 0.121 | 0.062 | **0.250** |
| TrafficBarrel | 0.138 | 0.219 | **0.338** |
| Concrete_Barricade01 | 0.149 | 0.158 | **0.235** |
| PE_Firewall_Orange | 0.116 | 0.209 | **0.580** |

- PP0의 전체 점수에는 `concrete-barrier` center recall `0.626`이 크게 기여한다.
- PP0도 나머지 다섯 asset의 observable recall은 `0.12~0.17`로 낮다.
- PP1은 CargoBox를 사실상 검출하지 못하고 긴 barrier류도 낮은 threshold에서
  Pedestrian으로 자주 오분류한다.
- S1g는 compact·barrier asset recall이 가장 높지만 background false positive가
  많다.

## Semantic audit와 oracle painting

Base3 LRASPP를 OpenCV, short side 384 조건으로 검사했다.

- LiDAR-visible obstacle point의 Obstacle probability 중앙값: `0.578`.
- Obstacle argmax fraction 중앙값: `0.663`.
- 거리 `7~16 m`에서는 probability 중앙값이 `0.83~0.93`이다.
- `16 m+`에서는 중앙값이 `0.026`으로 급락한다.
- asset별 Obstacle probability 중앙값:
  - CargoBox `0.136`.
  - PE_Drum `0.360`이며 point 수도 중앙값 2개뿐이다.
  - TrafficBarrel `0.581`.
  - Concrete_Barricade01 `0.904`.
  - PE_Firewall_Orange `0.950`.

PP1에 GT obstacle painting을 넣은 결과:

- 전체 recall@1m `0.038 -> 0.044`.
- LiDAR 5점 이상 recall@1m `0.036 -> 0.052`.

따라서 Base3 LRASPP는 CargoBox와 원거리에서 약점이 있지만, PP1 실패의 주원인은
segmentation 하나가 아니다. PP1 detector confidence calibration과 3D
shape/localization 일반화가 함께 병목이다.

## 결론

- 이 bag은 이전 obstacle3보다 대회 장애물 판단에 적합하다.
- 현재 score 0.10 production 후보 중에는 PP0가 가장 안전하다.
- S1g는 obstacle threshold를 약 0.22로 올리면 visible recall/F1이 좋지만, 정지
  Unknown이 planning을 막는 구조에서는 FP 비용을 먼저 검증해야 한다.
- PP1을 obstacle production으로 승격하지 않는다. threshold 0.02도 충분한 해결이
  아니다.
- 어느 모델도 다섯 유효 asset에서 높은 recall을 내지 못했다. 다음 개선은 전체
  obstacle 증량보다 `CargoBox/PE_Drum/TrafficBarrel`의 online 거리·가림·실제
  camera-LiDAR 조건을 겨냥한 targeted 학습과 class calibration이 우선이다.

## 산출물

- 공통 평가 디렉터리:
  `artifacts/evaluation/pointpainting/260912_base3_obstacle_benchmark/`
- asset별 평가기:
  `base_ws/src/tracking_prediction_evaluation/scripts/evaluate_static_obstacle_assets.py`
- 진단 전용 OpenPCDet proposal-floor override:
  `POINTPAINTING_MODEL_SCORE_THRESHOLD`.
  - 기본 `-1`은 config를 그대로 유지한다.
  - production 기본 동작은 바뀌지 않는다.
