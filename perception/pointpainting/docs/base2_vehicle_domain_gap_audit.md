# Base2 Vehicle domain-gap audit.

## 목적.

- Base2 Train·Val과 고정 실주행 rosbag을 동일한 기준으로 비교함.
- Vehicle GT 하나를 하나의 표본으로 사용함.
- 범위는 LiDAR 기준 30 m임.
- 방향은 전방·후방 각각 ±45도이며 나머지를 좌·우측으로 분류함.
- Online Recall은 S1 E38의 center-distance `Recall@1m`임.
- Train은 분포만 계산하며 detector 성능값은 없음.

## 전체 조건 비교.

| 조건 | Base2 Train | Base2 Val | 신로터리 | 고주로1 |
|---|---:|---:|---:|---:|
| Vehicle GT 수 | 1,697 | 325 | 315 | 402 |
| 좌측 | 8.3% | 7.7% | **19.4%** | 11.9% |
| 우측 | 5.3% | 2.2% | 0.0% | **7.7%** |
| 후방 | 30.3% | 17.8% | 25.1% | **50.5%** |
| 20–30 m | 32.6% | **51.4%** | 42.5% | 31.8% |
| Ego yaw-rate ≥5 deg/s | 23.7% | 16.6% | **43.5%** | 27.4% |
| LiDAR point <20 | **0.5%** | 0.0% | **14.0%** | 9.0% |

- Base2는 단순히 Vehicle 수가 적은 데이터셋은 아님.
- 가장 큰 차이는 실주행에서 저점밀도 표본이 크게 늘어난다는 점임.
- 신로터리는 좌측 Vehicle과 고회전 Ego가 동시에 나타나는 비율이 특히 높음.
- 고주로1은 후방·우측 Vehicle 비율이 Base2 Val보다 높음.

## 방향별 S1 E38 Recall@1m.

| 방향 | Base2 Val | 신로터리 | 고주로1 |
|---|---:|---:|---:|
| 전방 | 100.0% | 87.4% | 96.7% |
| 좌측 | 92.0% | **54.1%** | 87.5% |
| 우측 | 100.0% | 표본 없음 | **67.7%** |
| 후방 | 98.3% | 96.2% | 98.0% |

- 로터리의 주 약점은 전체 Vehicle이 아니라 좌측 Vehicle임.
- 고주로의 주 약점은 표본 수는 작지만 우측 Vehicle임.
- 전·후방 Vehicle은 현재 S1에서도 비교적 안정적임.

## 거리·점밀도별 S1 E38 Recall@1m.

| 조건 | Base2 Val | 신로터리 | 고주로1 |
|---|---:|---:|---:|
| 0–10 m | 100.0% | 표본 없음 | 100.0% |
| 10–20 m | 97.5% | 86.7% | 93.5% |
| 20–30 m | 100.0% | 79.1% | 97.7% |
| LiDAR point <20 | 표본 없음 | **63.6%** | 94.4% |
| LiDAR point 20–49 | 100.0% | 85.8% | 96.6% |
| LiDAR point ≥50 | 98.6% | 86.8% | 93.1% |

- Base2 Val에는 20 point 미만 Vehicle이 한 개도 없음.
- 신로터리의 20 point 미만 Vehicle은 44개이며 Recall이 63.6%임.
- 20–30 m 자체보다 거리 증가와 측방 관측·회전·희소 point가 결합되는 것이 문제임.

## 복합 조건.

| 조건 | Base2 Train | Base2 Val | 신로터리 | 고주로1 |
|---|---:|---:|---:|---:|
| 좌측 + yaw-rate ≥5 deg/s 비율 | 2.6% | 1.8% | **15.9%** | 5.0% |
| 위 조건 GT 수 | 44 | 6 | 50 | 20 |
| 위 조건 S1 Recall@1m | — | 66.7% | **48.0%** | 70.0% |
| 좌측 + 20–30 m 비율 | **0.0%** | **0.0%** | **7.9%** | 0.0% |
| 위 조건 S1 Recall@1m | — | 표본 없음 | 60.0% | 표본 없음 |

- 신로터리의 핵심 조건은 Base2 Train보다 약 6.1배 자주 나타남.
- Base2 Train·Val에는 좌측 20–30 m Vehicle이 한 개도 없음.
- Val mAP가 높아도 신로터리 성능을 대표하지 못한 직접적인 근거임.

## 판정.

- Base2 전체 설계가 실패한 것은 아님.
  - 전·후방과 일반 거리 조건은 S1에서 비교적 안정적임.
  - 실제 빈 배경 오검출도 S0 대비 S1에서 크게 줄었음.
- Hard-negative가 너무 많아 Vehicle을 못 배운 것이 주원인은 아님.
  - Train 5,509 frame 중 30 m Vehicle이 있는 frame은 약 29.8%임.
  - 완전 empty frame은 약 19.2%임.
- 주원인은 중요한 positive condition의 부족임.
  - 로터리 좌측·회전·희소 point 조합.
  - 고주로 우측 측방 조합.
- 따라서 다음 수집은 전체 Base2를 다시 만드는 방식보다 해당 positive slice를
  소규모로 보강하는 방식이 적합함.

## 다음 계획: Vehicle targeted add-on V1.

- 첫 규모는 96 clip, 768 frame으로 제한함.
- 8-frame temporal metadata 형식은 Base2와 동일하게 유지함.
- Clip 단위로 Train 72 / Val 12 / Test 12를 분리함.
- 최종 test는 생성기 Test보다 고정 실주행 rosbag을 우선함.

| 구성 | Clip | Frame | 목적 |
|---|---:|---:|---|
| 로터리 좌측·고회전 positive | 56 | 448 | 좌측 10–30 m, Ego yaw-rate 5–20 deg/s, 희소 point 조건 보강. |
| 로터리 일반 control | 16 | 128 | 전·후방과 저회전 구간 회귀 방지. |
| 고주로 우측 측방 positive | 16 | 128 | 추월·복귀 중 우측 5–25 m Vehicle 보강. |
| 일반 전·후방 control | 8 | 64 | 기존 강점과 confidence calibration 유지. |

- 실제 차로 위 pose와 대회 route만 사용함.
- 특정 K5나 한 spawn point에 고정하지 않고 Vehicle asset·거리·상대속도를
  clip 단위로 분산함.
- 단순 empty/hard-negative frame은 이번 add-on에 추가하지 않음.
- 주 실험 S2는 S1과 같은 모델·증강·seed·optimizer로 from-scratch 학습함.
- S1 E38 fine-tuning은 빠른 진단용 보조 실험으로만 사용하며 최종 비교 기준으로
  쓰지 않음.

### S2 통과 기준.

- 신로터리 좌측 Recall@1m: `54.1% -> 70% 이상`.
- 신로터리 좌측+yaw-rate 5 deg/s 이상 Recall@1m: `48.0% -> 68% 이상`.
- 고주로 우측 Recall@1m: `67.7% -> 80% 이상`.
- 신로터리·고주로 전체 center mAP가 S1보다 개선되거나 최소 `-0.01` 이내임.
- Base2 Val 전체 center mAP와 Pedestrian·Obstacle AP 하락이 각각 `0.01`
  이내임.
- Tracking recall, yaw 반전율, Map-CV 1–3초 p50/p90이 S1보다 악화되지 않음.

- 위 조건이 실패하면 같은 slice를 더 수집하지 않음.
- 그 경우 miss 표본의 camera/LiDAR 시각차, semantic painting, point 수와 score를
  직접 분석해 데이터 부족 가설을 재검토함.

## 재현 산출물.

- Script: `perception/pointpainting/tools/audit_base2_vehicle_difficulty.py`.
- Machine-readable result:
  `artifacts/detection3d/pointpainting/analysis/base2_s1_vehicle_difficulty/vehicle_difficulty_audit.json`.
- Per-object records:
  `artifacts/detection3d/pointpainting/analysis/base2_s1_vehicle_difficulty/vehicle_difficulty_records.jsonl`.
