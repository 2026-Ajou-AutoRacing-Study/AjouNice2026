# 새 팀원 rosbag과 기존 평가 비교·오차 분석

상태: EVIDENCE. 2026-10-04. Production E25 유지.
**대응 비교 표의 최신 기준은 [현재 계약으로 재실행한 비교](261004_E25_OLD_NEW_CURRENT_CONTRACT_COMPARISON.md)다.**
아래 기존 수치는 9/18 historical baseline 보존용이며, 같은 현재 파이프라인 비교에는 최신 표를 사용한다.
새 결과는 [기본 제원→geometry 복구 후 결과](261004_VEHICLE_STATUS_SIZE_GT_RECOVERY.md)를 사용한다.
모델 재학습이나 추가 추론 없이 기존 JSON과 저장된 출력의 지표를 조사했다.

## 먼저 비교의 한계

- 같은 E25와 class threshold 0.40/0.15/0.10, Center AP 0.5/1/2/4m 평균을 비교한다.
- 기존 9/18 결과는 generic center 보정, 당시 tracker와 지도 v68의 결과다.
  새 결과는 asset geometry, 현재 tracker, 지도 v85와 MR yaw 계약을 사용했다.
  따라서 표의 차이는 모델 개선율이 아니며 bag·GT·파이프라인 차이가 함께 포함된다.
- 기존 고주로는 합류/순항을 각각 녹화했다. 새 H bag은 합류부터 순항까지 연결된 주행이다.
  새 7-bag에는 development H01과 training_candidate H05가 포함된다.
  독립 판단의 우선 기준은 미리 지정된 locked 5-bag이다.
- 새 고주로 detection 후보 frame의 90.82%를 사용한다. 나머지는 근처 GT의
  geometric 크기를 확정할 수 없어 제외했다. 이 부분집합이 전체보다 쉽거나 어려운지는
  확인되지 않았으므로 전체 성능으로 일반화하지 않는다.
- 새 detector 재생은 0.5배속이며 출력 message coverage는 높았다. 실시간 full-stack
  frame drop, localization 오차, planning 회피 성공률을 검증한 결과는 아니다.
- 기존 자료의 수치는 [E25 선정 기록](260918_base3_e40_production_evaluation_results.md)과
  보존된 `bf_e40fresh_e25_class_thresholds_040_015_010_260918` 및
  `bf_e40fresh_e25_tracking_mapcv_260918` JSON에서 확인했다.

## 1. Detection 미션별 대응

| 미션 | 기존 E25 Center AP | 새 E25 Center AP | 해석 |
|---|---:|---:|---|
| 고주로 합류·순항 | 합류 57.00%, 순항 77.16% | 7-bag 85.78%, locked 5-bag 89.62% | 새로운 혼합 경로에서 차량 인지는 대체로 잘 유지된다 |
| 로터리 | 75.65% | 3-bag 83.01% | 새로운 로터리 차량에서도 큰 붕괴는 없다 |
| 보행자 | 횡단 99.99%, 거리별 98.89% | 3-bag 62.98% | 이전 거의 100%가 새 주행에 그대로 일반화되지 않는다 |
| 장애물 | 왕복 60.02%, multi-asset 64.72% | 3-bag 62.80% | 전반적인 약점이 여전히 남는다 |

보행자/장애물의 AP와 tracker F1은 매칭 거리·시점·계산 방식이 달라 서로 모순이 아니다.
Tracker F1이 높다고 raw detector의 엄격한 0.5m AP까지 좋다는 뜻은 아니다.

## 2. Tracking과 prediction

| 범위 | Track F1 | Speed MAE m/s | 2D AVE m/s | FDE@1/2/3s m |
|---|---:|---:|---:|---|
| 기존 차량 3-bag: 로터리+합류+순항 | 93.91% | 0.568 | 0.725 | 1.085 / 2.443 / 4.491 |
| 새 고주로 locked 5-bag | 96.30% | 0.520 | 0.622 | 0.625 / 1.046 / 1.530 |
| 새 고주로 전체 7-bag | 96.67% | 0.502 | 0.610 | 0.670 / 1.059 / 1.522 |
| 새 로터리 3-bag | 95.89% | 0.397 | 0.607 | 0.822 / 2.023 / 3.923 |
| 새 보행자 3-bag | 91.63% | 0.356 | 0.510 | 0.977 / 2.180 / 3.836 |
| 새 장애물 3-bag | 91.20% | ≈0 | ≈0 | 0.325 / 0.328 / 0.332 |

기존 고주로 단독 FDE@3s는 합류 5.619m, 순항 3.623m였다. 기존 로터리는
F1 94.55%, speed MAE 0.402m/s, AVE 0.701m/s, FDE@3s 4.232m였다.
새 고주로는 수치상 훨씬 좋지만, 새 로터리는 미래 경로 오차가 여전히 약 4m이다.
장애물 속도 ≈0은 강제 정적 정책의 결과이며 학습한 velocity 정확도의 증거가 아니다.
새 차량 ID switch는 고주로/로터리 모두 0, 보행자는 1, 장애물은 7이었다.
긴 미관측 구간의 새 ID 생성과 평가기의 ID-switch count는 같은 개념이 아니다.

## 3. 차량이 잘 나온 이유와 남은 hard case

확인된 근거:

- 새 고주로 AP 거리-bin macro는 0–10m 86.03%, 10–20m 87.59%, 20–30m 82.41%다.
  이번 차량 성능은 가까운 객체만 보고 얻은 숫자는 아니다.
- 대부분의 H02/H04 run은 1m AP가 약 96–97%이며 center TP 평균 오차가 0.20–0.22m다.
  비교적 정확한 위치 관측을 지속적으로 얻을 수 있었다.
- 원본의 거리/receipt-time 속도적분 비율은 0.968–1.000이었다. 예전 로컬 full-stack의
  큰 시간축 속도 불일치와 같은 현상은 발견되지 않았다. 시간축 조건이 더 양호하다는
  근거이지, 성능 상승에서 차지한 비중을 분리한 A/B 증거는 아니다.
- 고주로 FDE@3s의 bag-macro는 top-1 1.522m, GT가 사후 선택하는 best-of-K 1.484m다.
  대안 경로 선택만 바꿔 줄일 수 있는 잔여 오차는 작다.
- 시작 위치 오차를 제거한 origin-aligned FDE@3s도 1.366m다.
  잔여 오차의 상당 부분은 단순 BBox center offset보다 미래 motion에 남아 있다.

잘 나온 숫자만으로 고속 대응을 확정하지 않는다:

- H03(최고 Ego 101.7km/h)은 AP 74.35%, tracker speed MAE 0.837m/s,
  speed-error p95 3.228m/s이다. H04(최고 92.5km/h)는 AP 94.44%지만 FDE@3s 2.584m다.
  Ego 최고 속도 하나로 detector/velocity 성능을 설명할 수 없다.
- H03 detection AP는 1m 기준 61.1%인데 2m/4m는 각각 93.7%다.
  H05는 0.5/1/2/4m AP가 1.2/55.8/98.6/98.6%이다.
  이 두 사례는 주로 검출 자체 소실보다 center 정밀도에서 엄격한 기준을 놓치는 양상이다.
- H05는 보정 가능한 범위의 GT가 총 44 box-observation뿐이다.
  넓은 분포의 대표값으로 해석하지 않는다. H05는 locked test도 아니다.
- H03의 GT ambiguity 제외와 토픽 공백도 기록되어 있다. 높은 상대속도·시점 정합·부분
  가림 중 무엇이 center 오차의 직접 원인인지는 이 집계만으로 확정하지 않았다.

## 4. 로터리: 탐지보다 경로 선택 문제

새 로터리 top-1 FDE@3s는 3.923m지만 best-of-K는 2.201m이다.
현재 origin-aligned top-1도 3.882m라서 시작 BBox 위치만 정확하게 해도 문제가 남는다.
미래 GT heading 기준 횡오차 평균은 bag별 1.424–2.059m이며 고주로의 0.005–0.203m보다 크다.
따라서 지도 경로 후보의 순위/분기 선택과 회전 motion을 우선 의심할 근거가 있다.
best-of-K는 GT를 보고 고른 oracle 지표이므로 production 달성 성능으로 쓰지 않는다.
이는 velocity 하나만 개선하면 로터리 FDE가 해결된다는 증거가 아니다.

## 5. 보행자: 새 분포에서 원거리 누락과 center 정밀도 문제

| 새 보행자 거리 | AP bag macro | GT box-observation | 1m TP / FN |
|---|---:|---:|---|
| 0–10m | 85.35% | 149 | 144 / 5 |
| 10–20m | 69.24% | 107 | 100 / 7 |
| 20–30m | 6.96% | 88 | 14 / 74 |

기존 횡단 bag의 GT 289개는 모두 10–20m였고, 거리별 bag도 0–20m에만 GT가 있었다.
따라서 이전 99%는 20–30m 보행자 능력을 검증하지 않은 결과였다.
새 run 전체의 1m FP는 0/1/0개이고 FN은 29/23/24개로, 무차별 오검출보다 누락이 크다.
또한 새 전체 0.5m AP는 26.6–28.1%인데 1m AP는 71.1–77.8%다.
가까운 객체에서도 center 정밀도 저하가 있어, 모든 하락을 원거리 GT 추가만으로 설명하지 않는다.

보행자 3초 FDE는 origin alignment 후에도 3.680m이며 best-of-K 이득이 없다.
Tracker yaw 평균 오차는 100.8–122.1°이나 mesh heading과 이동 방향은 다를 수 있다.
현재 AVE는 heading projection이 아닌 실제 XY 위치 차분으로 평가했으므로, 이 yaw 차이만으로
속도 GT가 틀렸거나 yaw 수정 하나로 prediction이 해결된다고 단정하지 않는다.
기존 bag에 대한 같은 pedestrian tracking/prediction 기준선은 없으므로 개선/퇴행률을 만들지 않는다.

## 6. 장애물: 중거리 양호, 근접·원거리 취약

새 장애물 AP 거리-bin macro는 0–10m 49.01%, 10–20m 86.78%, 20–30m 48.35%다.
기존 왕복도 0–10m AP 42.48%였으므로 근접 취약성이 새롭게만 생긴 것은 아니다.
새 obstacle의 1m FP는 run별 10/2/14개, FN은 225/168/244개다.
검출 누락이 주된 약점이며 coasting으로 전부 해결할 수 있는 상태라고 보지 않는다.
부분 가림·얇은 면·시야에서 벗어나는 통과 구간이라는 가설은 있지만 asset/시점별 분석 전
이를 이번 FN의 확정 원인으로 쓰지는 않는다.

3초 FDE 0.332m는 정적 정책에 따른 현재 위치 오차가 유지된 값이다.
origin-aligned FDE는 0이므로 이를 prediction 모델의 우수한 일반화로 홍보하면 안 된다.
ID switch 7건은 O01_163803/O01_163859/O02_164012에서 각각 1/4/2건이다.
첫 O01은 collision payload가 있는 녹화이며, 동일 파이프라인 자율주행의 성공 사례로
집계하지 않는다. 이 데이터는 사람이 운전한 raw bag을 오프라인 재생한 인지 평가다.

## 7. P7 temporal 연구 결과의 의미

새 locked 5-bag에서 E25→P7은 AP 89.62→89.93%, speed MAE 0.520→0.514m/s,
AVE 0.622→0.619m/s, FDE@3s 1.530→1.513m다. 실용적으로 매우 작은 차이다.
두 경우 모두 tracker detector-velocity 융합 OFF라서 이 차이는 주로 spatial detection
입력 차이에 따른 것이며 temporal velocity 융합 이득이라고 해석할 수 없다.

P7 detector-only 종방향 MAE 0.780m/s는 기존 개발 bag에서 기록한 약 0.779/0.811m/s와
비슷한 수준이다. 심각한 일반화 붕괴를 주장할 근거도, 목표 0.5m/s를 달성했다고 할 근거도 없다.
서로 다른 TP 집합과 GT 계약이므로 paired estimator 검정은 아니다.
새 detector-only AVE는 1.204m/s로 위치 기반 tracker 0.619m/s보다 크다.
강제 velocity 융합·production 교체를 할 근거는 아직 부족하다.

## 결론과 우선순위

1. E25 + position-only tracker를 유지한다. 새 자료에서 차량은 대체로 잘 유지됐다.
2. 고주로 평균값보다 H03 속도 tail, H04 긴 horizon 종오차를 다음 진단 대상으로 삼는다.
   locked bag으로 반복 튜닝하면 더 이상 독립 최종 test로 부르지 않는다.
3. 로터리는 후보 경로 순위/분기와 곡선 운동을 분리 진단한다. velocity만의 문제로 몰지 않는다.
4. 보행자는 20–30m 누락과 0.5m center 정밀도, 장애물은 근접 누락/ID 단절을 별도 과제로 둔다.
5. 남은 9.18% GT 모호 구간을 복구하거나 제외 집합의 난이도를 감사해야 고주로 전체 결과를
   주장할 수 있다. 새로운 independent bag은 이후 최종 검증용으로 따로 유지한다.

산출물: `artifacts/evaluation/bevfusion/teammate_manual_261004/aggregate_status_size_v2.json`,
각 case의 `summary_status_size_v2.json` 거리-bin/AP threshold/TP 품질,
`pipeline/summary_status_size_v2.json` origin-aligned·best-of-K·GT-heading error decomposition.
원본 시간축/속도/충돌 정보는 [raw 감사](261004_TEAMMATE_MANUAL_ROSBAG_RAW_AUDIT.md)를 따른다.
