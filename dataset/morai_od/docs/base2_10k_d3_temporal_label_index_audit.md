# Base2-10K D3 temporal label·index audit

## 결론.

- Base2-10K 원본 10,504 frame은 수정하지 않았음.
- 1,313개 clip에서 3-sweep causal sample 7,878개를 생성함.
- 각 8-frame clip에서 `f02`부터 `f07`까지 6개 target을 사용함.
- 실제 학습은 아직 시작하지 않았음.
- D3 산출물은 PointPainting temporal loader를 붙이기 위한 index·label 계약임.
- 출력 위치는 `artifacts/datasets/morai_temporal_base2_10k_d3`임.

## 용어와 sample 구성.

- Clip.
  - 동일 scene에서 약 12 Hz로 연속 촬영된 8개 frame 묶음임.
- Sweep.
  - 한 시점의 LiDAR와 4-camera 관측임.
- Temporal sample.
  - 과거 2 sweep과 현재 target sweep을 묶은 학습 단위임.

```text
oldest sweep        middle sweep        target sweep
약 -0.17 s          약 -0.08 s          0.00 s
      \                  |                  /
       +------ target LiDAR 좌표계로 정합 ------+
```

- 입력 순서는 oldest-to-current임.
- 미래 sweep은 입력에 들어가지 않음.
- 실제 `time_lag_sec`를 저장하며 고정 12 Hz로 간주하지 않음.
- Sweep 간격 p50은 0.0815초, p95는 0.1004초임.
- 0.04~0.14초 밖의 불규칙 temporal sample은 7,878개 중 6개임.
- 6개는 삭제하지 않고 `temporal_input_valid=false`로 표시함.

## Split 결과.

| Split | Clip | 3-sweep sample |
| --- | ---: | ---: |
| Train | 919 | 5,514 |
| Val | 197 | 1,182 |
| Test | 197 | 1,182 |
| 합계 | 1,313 | 7,878 |

- Clip 단위 split을 그대로 사용함.
- 동일 clip의 frame이 서로 다른 split으로 이동하지 않음.
- Sample ID 중복 0개, split overlap 0개, future input 0개임.

## Sweep 간 Ego motion alignment.

- 과거 sweep point를 현재 target LiDAR 좌표계로 옮기는 행렬은 다음과 같음.

```text
T_target_ego_from_sweep_ego
    = T_target_ego_from_global × T_global_from_sweep_ego

T_target_lidar_from_sweep_lidar
    = T_lidar_from_ego
    × T_target_ego_from_sweep_ego
    × T_ego_from_lidar
```

- SaveSensorData에는 정확한 point time이 없으므로 scan 내부 deskew는 하지 않음.
- 이번 D3에서 구현한 것은 sweep 간 Ego motion alignment임.

### 정합 검증.

| 검증 대상 | 정합 전 p50 | 정합 후 p50 | 변화 |
| --- | ---: | ---: | ---: |
| ObjectStatus 상대 위치 | 0.835 m | 사실상 0 m | 좌표식 sanity check 통과 |
| LiDAR `Instance.txt` GT box center 310개 | 1.346 m | 0.069 m | 94.9% 감소 |

- 두 번째 결과가 실제 저장된 LiDAR sensor-frame GT를 이용한 주 검증임.
- 동일 unique ID의 oldest/target box center를 비교했음.
- 정합 후 p90은 0.141 m, p95는 0.203 m임.
- 남은 차이는 trigger 시각과 실제 sensor capture 시각 차이, rolling scan, actor motion, 저장 시각 양자화가 섞인 값임.

### Full-cloud nearest-neighbor 진단의 주의점.

- 라벨 없는 전체 cloud NN은 정합 전 p50 0.129 m, 정합 후 0.511 m로 악화됨.
- 이는 행렬 방향이 틀렸다는 뜻이 아님.
- 도로 평면처럼 반복되는 조밀한 표면은 정합하지 않아도 가까운 이웃이 항상 존재함.
- FOV·가림·rolling scan 때문에 두 sweep point 사이에 one-to-one correspondence도 없음.
- 따라서 full-cloud NN은 참고값으로만 유지하고 release gate로 쓰지 않음.
- Sensor GT box center 검증을 alignment acceptance 기준으로 사용함.

## Velocity GT 좌표와 단위 검증.

- 개인용 MRP 메시지 정의에서 `ObjectStatus.velocity` 단위는 km/h임.
- `EgoVehicleStatus.velocity` 단위는 m/s임.
- 실제 Base2 위치 변화와 비교하면 ObjectStatus XY는 world vector가 아니라 actor-local vector로 해석해야 맞음.
- ObjectStatus를 3.6으로 나눈 뒤 actor heading으로 world에 회전함.

| 해석 | Moving object vector error p50 | p95 |
| --- | ---: | ---: |
| Actor-local → heading 회전 | 0.276 m/s | 1.459 m/s |
| Raw XY를 world로 오해 | 7.876 m/s | 21.323 m/s |

- 방향 오차 p50은 1.36도, p95는 7.71도임.
- 따라서 actor-local 회전 해석을 D3 canonical velocity GT로 확정함.
- Vehicle raw-derived speed 차이 p50은 0.023 m/s임.
- Vehicle p95 1.477 m/s는 주로 clip 끝에서 위치 미분 표본이 짧아지는 tail임.
- Simulator raw state를 noisy endpoint fit으로 교체하지 않고, disagreement label의 weight만 낮춤.

## Velocity label source 우선순위.

1. `static_zero`.
   - Static family 또는 obstacle이고 raw·위치 변화가 모두 정지를 확인한 경우임.
2. `morai_raw_local_rotated`.
   - Timing sidecar에서 target trigger에 가장 가까운 ObjectStatus를 사용함.
   - 단위와 좌표계를 변환한 simulator raw GT임.
3. `derived_diagnostic`.
   - 3~5 frame world position을 시간에 대해 선형 fitting한 값임.
   - Raw GT 검증용이며 production velocity loss에는 넣지 않음.
4. `invalid`.
   - ID 연속성이나 raw state가 없어 velocity loss에서 제외함.

| Source | Label 수 | Velocity loss 사용 |
| --- | ---: | --- |
| `static_zero` | 15,563 | 사용 |
| `morai_raw_local_rotated` | 5,206 | 사용 |
| `invalid` | 5 | 제외 |

- 유효 velocity label은 20,769 / 20,774개, 즉 99.976%임.
- Class별 유효율은 Obstacle 100%, Pedestrian 100%, Vehicle 99.94%임.

## Label quality mask.

- `velocity_valid`.
  - Velocity regression loss 사용 가능 여부임.
- `velocity_loss_weight`.
  - 정상 label은 state age와 raw-derived consistency에 따라 0.25~1.0임.
  - Raw-derived disagreement 59개는 0.25로 낮춤.
  - Invalid 5개는 0임.
- `quality.identity_continuous`.
  - 같은 unique ID가 최소 3 frame 유지됐는지 나타냄.
- `quality.timestamp_valid`.
  - Nearest ObjectStatus age가 0.1초 이하인지 나타냄.
- `quality.raw_plausible`.
  - 변환된 raw speed가 유한하고 60 m/s 이하인지 나타냄.
- `quality.position_continuous`.
  - Position fit이 비정상 점프를 보이지 않는지 나타냄.
- `quality.raw_derived_consistent`.
  - Raw와 position-derived 속도의 크기·방향이 합의하는지 나타냄.
- Velocity가 invalid여도 box class·geometry label은 계속 사용할 수 있음.

## Timestamp 검증.

- Embedded pre-trigger ObjectStatus snapshot만 사용하지 않음.
- 각 clip의 high-rate timing sidecar에서 trigger monotonic time에 가장 가까운 ObjectStatus를 선택함.
- 선택된 ObjectStatus age는 p50 3.59 ms, p95 8.93 ms, max 26.34 ms임.
- 이전 D0에서 보였던 수백 ms짜리 embedded-tail 문제는 D3 velocity label에서 제거됨.
- 다만 SaveSensorData가 정확한 camera exposure·LiDAR point timestamp를 주지는 않음.
- 이 값은 trigger-state 정렬 품질이지 exact sensor-time GT라는 뜻은 아님.

## D3 출력.

| 파일 | 역할 |
| --- | --- |
| `temporal_samples.jsonl` | 3-sweep sensor path, 실제 lag, Ego pose, 상대 변환, temporal mask. |
| `velocity_labels.jsonl` | Target-frame object velocity와 quality mask. |
| `temporal_schema.json` | 좌표계·단위·source priority 계약. |
| `sample_summary.csv` | Sample별 label 수와 유효 label 수. |
| `splits/*_samples.txt` | Split별 sample ID. |
| `audit.json` | 전체 machine-readable audit. |
| `audit.md` | 산출물 안의 간단한 release report. |
| `*.png` | Lag·velocity 좌표 가설·정합 분포 plot. |

- 필요한 4-camera intensity, LiDAR intensity, LiDAR Instance GT reference 63,024개를 전수 검사함.
- 누락 reference는 0개임.
- 삭제된 Semantic/Depth 경로는 D3 index에 넣지 않음.
- 전체 D3 index·audit 크기는 약 77 MB이며 sensor 원본을 복사하지 않음.

## 재현 명령.

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/build_temporal_training_index.py \
  --dataset /home/luv/morai_cap/morai_temporal_sequence_v2_base2_10504_final \
  --output-dir artifacts/datasets/morai_temporal_base2_10k_d3 \
  --overwrite

python3 -m unittest \
  dataset.morai_od.tests.test_temporal_training_index
```

## D4로 넘기는 계약.

- D4 loader는 `temporal_input_valid=true` sample을 기본 사용함.
- 과거 point는 `lidar_to_target_lidar`로 target 좌표계에 정합함.
- 각 point 또는 sweep feature에 실제 `time_lag_sec`를 전달함.
- Velocity loss는 `velocity_valid=true`만 사용하고 `velocity_loss_weight`를 곱함.
- `derived_diagnostic`은 별도 ablation을 명시적으로 켜지 않는 한 target으로 사용하지 않음.
- Scan 내부 방위각 근사 deskew는 production에 넣지 않음.
- 실제 학습 전 D4 smoke에서 3-sweep loading, painting 순서, augmentation의 sweep-consistency를 추가 검증해야 함.
