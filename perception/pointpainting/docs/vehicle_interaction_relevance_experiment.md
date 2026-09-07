# Vehicle interaction-relevance experiment.

## 상태와 목적.

- 이 문서는 production 필터를 설계하는 문서가 아님.
- Full-360 rosbag OD 점수가 실제 대회 Planning 중요도를 과도하게 왜곡하는지
  검증하는 offline oracle 실험임.
- Runtime topic, detector threshold, tracker, predictor는 변경하지 않았음.
- 비교 모델은 Legacy E65, S1 E38, S1b E30임.
- 입력은 신로터리와 고주로1 고정 rosbag임.

## 가설 1.

- 전체 OD miss의 상당 부분은 이미 지나쳤거나 Ego 경로와 만나지 않는
  peripheral Vehicle에서 발생함.
- 따라서 Full-360 mAP만으로 production 유용성을 판단하면 실제 위험도를
  과장할 수 있음.

## 1차 실험과 기각된 정의.

- 처음에는 실제 Ego·Vehicle의 같은 미래 시각 간 최소거리와 안전하게 실현된
  도달 시간차를 함께 사용함.
- 이 정의에서는 로터리 conflict-critical 표본이 0개가 됐음.
- Ego가 위험 차량 때문에 이미 감속·양보하면 실제 미래에는 가까워지지 않기
  때문임.
- 즉 안전하게 실현된 결과로 원래 위험도를 판단하는 post-treatment 오류가
  발생함.
- 이 정의는 기각함.

## 수정한 oracle 정의.

- 현재와 미래의 동시점 최소거리 외에, Ego와 Vehicle의 GT swept path를 서로
  다른 시각끼리도 비교함.
- 3초 또는 5초 안에 두 경로의 기하학적 최소거리가 3.5 m 이하이면
  conflict-critical로 분류함.
- 실제 도달 시간차는 진단값으로 보존하지만 critical 판정에는 사용하지 않음.
- 동시점 최소거리 8 m 이하도 critical로 포함함.
- 동시점·현재거리 15 m 이하 또는 swept-path 거리 5 m 이하는
  planning-relevant로 분류함.
- 나머지는 peripheral임.
- 이 threshold는 실험 protocol이며 production filtering 계약이 아님.

## 가설 1 결과.

### 3초 interaction horizon OD Recall@1m.

| Scene | Model | 전체 Recall | Critical GT | Critical Recall | Critical miss |
|---|---|---:|---:|---:|---:|
| 신로터리 | Legacy E65 | 84.8% | 109 | 93.6% | 7 |
| 신로터리 | **S1 E38** | 83.2% | 109 | **98.2%** | **2** |
| 신로터리 | S1b E30 | 74.6% | 110 | 84.5% | 17 |
| 고주로1 | Legacy E65 | 83.2% | 187 | 92.5% | 14 |
| 고주로1 | **S1 E38** | 94.0% | 186 | **97.3%** | **5** |
| 고주로1 | S1b E30 | 86.5% | 186 | 89.2% | 20 |

- S1은 Full-360 로터리 Recall이 Legacy보다 낮지만 3초 critical Recall은 더 높음.
- 신로터리 S1 miss 53개 중 3초 critical miss는 2개뿐임.
- 전체 점수 하락 대부분이 3초 안에 경로가 겹치지 않는 Vehicle에서 발생한다는
  가설을 지지함.

### 5초 sensitivity OD Recall@1m.

| Scene | Model | Critical GT | Critical Recall | Critical miss |
|---|---|---:|---:|---:|
| 신로터리 | Legacy E65 | 157 | 86.0% | 22 |
| 신로터리 | **S1 E38** | 157 | **89.2%** | **17** |
| 신로터리 | S1b E30 | 158 | 77.2% | 36 |
| 고주로1 | Legacy E65 | 221 | 89.1% | 24 |
| 고주로1 | **S1 E38** | 220 | **97.7%** | **5** |
| 고주로1 | S1b E30 | 220 | 89.5% | 23 |

- 5초로 늘리면 로터리 진입 4–5초 전에 보이는 좌측 18–30 m Vehicle이 critical로
  들어옴.
- S1의 critical Recall은 89.2%로 내려가므로 모든 성능 저하가 peripheral이라는
  강한 가설은 기각함.
- 다만 5초에서도 S1이 세 모델 중 가장 높음.
- Path threshold를 2.0 / 3.5 / 5.0 m로 바꿔도 신로터리 S1 Recall은
  89.0 / 89.2 / 89.9%로 결론이 유지됨.

## 가설 2.

- Peripheral OD miss는 tracker가 메우며 Planning 영향은 더 작음.
- Critical Vehicle의 Tracking coverage와 연속 outage가 충분하면 detector
  Full-360 miss를 우선적으로 보강할 필요가 없음.

## Tracking 결과.

### S1 E38 critical Tracking.

| Horizon label | Scene | Coverage | Center p90 | Speed p90 | 최대 outage |
|---|---|---:|---:|---:|---:|
| 3초 | 신로터리 | 97.7% | 0.598 m | 0.871 m/s | 0.20 s |
| 3초 | 고주로1 | 100.0% | 0.759 m | 0.877 m/s | 0.00 s |
| 5초 | 신로터리 | 90.1% | 0.847 m | 1.201 m/s | 0.80 s |
| 5초 | 고주로1 | 100.0% | 0.748 m | 0.919 m/s | 0.00 s |

- 3초 critical에서는 tracker가 거의 모두 메움.
- 5초 로터리 접근에서는 0.8초 단절이 남아 가설이 완전히 성립하지 않음.
- 5초 로터리 critical Tracking coverage는 Legacy 90.6%, S1 90.1%로 비슷함.
- S1은 center·speed p90이 Legacy보다 낮으므로 track이 존재할 때 상태 품질은
  더 좋음.

## Prediction 진단.

- 5초 critical label을 사용하되 Map-CV 출력은 1·2·3초로 평가함.
- 표는 CV-stable GT subset의 minFDE-K임.

| Scene | Horizon | Coverage | p50 | p90 |
|---|---:|---:|---:|---:|
| 신로터리 | 1초 | 90% | 0.61 m | 1.88 m |
| 신로터리 | 2초 | 94% | 1.07 m | 3.76 m |
| 신로터리 | 3초 | 100% | 1.24 m | 5.03 m |
| 고주로1 | 1초 | 100% | 0.47 m | 1.21 m |
| 고주로1 | 2초 | 100% | 0.79 m | 2.00 m |
| 고주로1 | 3초 | 100% | 1.34 m | 2.87 m |

- Critical track이 존재해도 3초 prediction p90 tail은 남음.
- 이는 detector Full-360 분포 문제와 별개인 Map-CV 속도·경로 tail임.
- 이번 OD targeted-data 판단에 prediction tail을 섞지 않음.

## 최종 판정.

- S1 E38 production comparator는 유지함.
- 고주로 Vehicle targeted add-on은 현재 근거로 보류함.
- 로터리 전체 좌측·후방 Vehicle add-on도 보류함.
- 유일하게 반복 검증할 가치가 있는 조건은 다음과 같음.
  - 로터리 진입 약 4–5초 전.
  - 좌측 18–30 m 접근 Vehicle.
  - Ego가 진입·회전을 시작하는 구간.
- 현재 한 rosbag의 한 상호작용에서 나온 표본이므로 바로 수집하지 않음.
- 팀원이 추가한 독립 로터리 bag에서도 같은 조건의 5초 critical Recall이 95%
  미만이거나 Tracking outage가 0.3초를 넘을 때만 add-on을 승인함.

## 조건부 후속 실험.

- 반복되면 24–40 clip, 192–320 frame만 수집함.
- 기존 768-frame 광범위 add-on 계획은 폐기함.
- 수집 대상은 로터리 진입 좌측 15–30 m 접근 Vehicle로 제한함.
- Vehicle asset, 초기 거리, 상대속도, Ego 접근 속도를 분산함.
- 특정 rosbag의 pose를 복제하지 않음.
- S2는 S1과 같은 설정으로 from-scratch 학습함.
- 통과 기준은 다음과 같음.
  - 독립 bag 5초 critical OD Recall 95% 이상.
  - Critical Tracking coverage 97% 이상.
  - Critical outage p95 0.2초 이하.
  - Base2 Val과 Full-360 rosbag AP 하락 0.01 이내.

## 산출물.

- Evaluator:
  `perception/pointpainting/tools/audit_vehicle_interaction_relevance.py`.
- Unit test:
  `perception/pointpainting/tests/test_vehicle_interaction_relevance.py`.
- 3초 결과:
  `artifacts/detection3d/pointpainting/analysis/vehicle_interaction_relevance_v2`.
- 5초 세 모델 비교:
  `artifacts/detection3d/pointpainting/analysis/vehicle_interaction_relevance_h5`.
- 5초 S1 prediction 결과:
  `artifacts/detection3d/pointpainting/analysis/vehicle_interaction_relevance_h5_s1`.
