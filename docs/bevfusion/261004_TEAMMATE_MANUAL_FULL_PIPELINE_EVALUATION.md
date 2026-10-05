# 팀원 수동주행 rosbag: E25 / Q1-P7 전체 파이프라인 평가

주의: 아래 차량 결과는 기존 size resolver를 사용한 최초 기록이다.
기본 제원과 geometric BBox 크기를 직접 비교한 오류가 확인되어,
최신 차량 결과는 [GT 대응 복구 및 재평가](261004_VEHICLE_STATUS_SIZE_GT_RECOVERY.md)를 따른다.
이 문서의 수치는 당시 결과의 보존용이며 production 성능 판단에 그대로 사용하지 않는다.

상태: EVIDENCE. 원본 감사는 [raw 감사](261004_TEAMMATE_MANUAL_ROSBAG_RAW_AUDIT.md)를 참고한다.
이번 결과로 production 모델·threshold·tracker·planning 설정을 변경하지 않는다.

## 실행과 계약

- 원본: `rosbags/eval/perception/260923_teammate_manual/`의 16개 bag.
- E25: 모든 미션 16개. Q1-P7: 학습 범위인 고주로 7개만 비교한다.
  두 모델의 대상 bag 모두 detector→tracker→predictor 실행 및 JSON 검증을 완료했다(총 23 case).
- E25 checkpoint: `artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth`.
- P7: P3 E1 checkpoint와 `bevfusion_base4_q1_p7_free_intercept_4cam_lidar_velocity.py`.
  P7 query-memory를 켜지만 tracker에는 detector velocity를 융합하지 않는다(mode 0).
  따라서 P7 detector velocity 오차와 P7 위치 기반 tracker 속도 오차는 별개 지표이다.
- 입력: 4 camera + LiDAR. 현재 각 모델 계약대로 scan 내부 deskew off.
- threshold Vehicle/Pedestrian/Obstacle `0.40/0.15/0.10`.
- 평가 범위: BBox-center와 Ego 사이 2D 거리 30m 이내. Tracker association 2m.
- Center AP: center-distance 0.5/1/2/4m AP 평균. FDE는 top-1, 모든 운동 구간 기준.
- Tracker CTRV, XY measurement noise 0.4m, maximum internal prediction age 0.5s.
  Production lifecycle/duplicate suppression은 유지했다. Map-CV 지도 v85.
- TF/ego pose는 recorded Ego 기반 GT로 재생한다. MR heading -1.347°는 한 번만 적용한다.
  이 평가는 localization·planning·controller의 end-to-end 주행 성능 검증이 아니다.
- 추론은 0.5배속, detector 출력 저장 후 tracker/predictor는 1배속·별도 ROS master로 실행한다.
  이는 실시간 frame loss를 최소화한 품질 평가이며 production Hz/latency 검증이 아니다.
  E25는 LiDAR 7,948개 중 detection message 7,947개를 기록했다. 초기 Ego 보간 실패 16개는
  별도로 집계한다. detection message coverage와 객체 검출 recall은 다른 지표이다.
  P7은 LiDAR 4,881개 중 detection message 4,878개를 기록했다.
- GT는 75-asset geometric BBox catalog 적용. Ped/Obstacle은 원본 metadata SHA와 일치하는
  시나리오 UNIQUEID/DataID에서 이름을 복원한다. Vehicle은 기존 size resolver를 사용한다.
- 긴 GT 공백을 넘지 않는다: trajectory 분할 gap 0.5s, interpolation bracket 0.15s.

## 미식별 GT와 두 평가표

기존 evaluator는 카탈로그 미식별 GT segment를 제외하지만, 해당 차량의 검출 결과가 FP로
남을 수 있다. 새 bag 중 일부에서 이 효과가 크게 나타났다. 이 값을 모델 오검출이나
일반화 실패로 곧바로 해석하면 안 된다.

1. `summary.json`, `pipeline/summary.json`: 기존 supported-GT 평가 방식 그대로 보존.
2. `summary_resolved_frames.json`, `pipeline/summary_resolved_frames.json`: 근처에 미식별
   GT가 있으면 그 **프레임 전체**를 제외한 보수적인 추가 평가. 추정 center/size를 만들지 않는다.
   pivot→center의 범위 경계 차이를 고려하여 raw pivot 35m 이내 미식별 GT를 검사한다.
   FP가 많은 frame을 score로 골라 버리는 방식이 아니라 GT 식별 여부만 사용한다.

두 모델에 같은 raw GT 조건을 적용한다. 제외된 프레임 수와 비율을 반드시 함께 제시한다.
두 번째 표도 전체 도로의 무조건 성능이 아니며, 미식별 asset catalog 보강이 필요하다.
미식별 GT가 없는 Ped/Obstacle은 두 표가 거의 동일하다.

## 속도와 예측 지표

2026-10-05 추가 검토: 아래 차량 지표는 기존 위치 미분 GT 계약이다.
[새 H01 동일 tracker 출력 GT A/B](261005_VEHICLE_VELOCITY_GT_REFERENCE_REVIEW.md)에서
메시지 속도 기준과 유의한 수치 차이가 확인됐다. 기존 결과는 보존하며, 메시지
velocity의 단위·좌표계·기준점 검증 후 별도 계약으로 재평가할 예정이다.

- Vehicle speed MAE는 기존 signed body-x 기준을 유지한다. 2D AVE는 map-frame 벡터 L2 오차이다.
- 보행자는 mesh heading과 보행 방향이 다르고 `/Object_topic.velocity`가 0인 bag이다.
  causal 위치 이력의 XY 미분을 사용하며 speed MAE는 벡터 크기 차이, AVE는 XY 벡터 차이다.
- Obstacle의 속도 0은 runtime stationary 정책이다. 거의 0의 AVE가 velocity head의 학습 성능을 뜻하지 않는다.
- FDE@1/2/3s는 현재 위치 오차를 포함한 top-1 미래 위치 오차이다. Vehicle CV-stable subset은 보조 지표이다.
  보행자의 CV-stable/oracle-map 보조 집계는 기존 heading-projection 기준이 남아 있으므로
  사용하지 않는다. 보행자는 full XY GT로 검증한 tracking 속도와 all-motion FDE만 보고한다.
  놓친 객체에는 FDE를 계산할 수 없으므로 tracking recall/prediction coverage를 함께 본다.
- 미션 macro는 bag별 평균의 비가중 평균이다. 서로 다른 bag의 p50/p95를 pooled quantile처럼
  평균내지 않는다. ID switch는 합계이며 긴 미관측 뒤 재등장한 ID 전체 개수가 아니다.

## 새 bag의 E25 결과

아래는 미식별 GT가 근처에 없는 프레임의 추가 평가이다. Bag macro이며 whole-drive 결과로
일반화하지 않는다. 고주로 detection frame은 4,120/4,872개(84.56%)를 사용했다.

| 미션 | bag 수 | Center AP | Track F1 | ID switch 합계 | Speed MAE m/s | 2D AVE m/s | top-1 FDE@1/2/3s m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 고주로 전체 | 7 | 84.24% | 95.52% | 0 | 0.542 | 0.627 | 0.739 / 1.193 / 1.746 |
| 고주로 evaluation_locked | 5 | 87.04% | 94.69% | 0 | 0.573 | 0.661 | 0.705 / 1.195 / 1.769 |
| 로터리 | 3 | 83.01% | 95.89% | 0 | 0.397 | 0.607 | 0.822 / 2.023 / 3.923 |
| 보행자 횡단 | 3 | 62.98% | 91.63% | 1 | 0.356 | 0.510 | 0.977 / 2.180 / 3.836 |
| scale=1 multi-obstacle | 3 | 62.80% | 91.20% | 7 | ≈0 | ≈0 | 0.325 / 0.328 / 0.332 |

3초 prediction coverage의 bag macro는 고주로 약 89.6%(locked 약 87.0%), 로터리 약 95.0%,
보행자 약 85.2%, 장애물 약 84.7%이다. FDE가 작은 것만으로 미검출 객체가 안전하게 처리된다고
해석하지 않는다.

원래 supported-GT/all-frame 집계의 E25 고주로 AP는 70.33%, tracking F1은 81.31%,
speed MAE는 0.490m/s, AVE는 0.593m/s이다. 이를 정상적인 전체 GT 평가로 부르지 않는다.
위 subset에서는 speed MAE가 오히려 0.542로 커진다. 제외 정책은 모든 지표를 좋게
보이게 만드는 선택이 아니다. 미식별 GT 및 실제 어려운 구간은 원본 결과로 계속 남긴다.

| 고주로 bag | 유지한 detection frame / 후보 | subset의 GT box 수 |
| --- | ---: | ---: |
| H01 161315 | 704 / 764 | 259 |
| H02 161616 | 724 / 724 | 469 |
| H02 161831 | 268 / 589 | 35 |
| H02 162108 | 779 / 779 | 401 |
| H03 162301 | 512 / 649 | 119 |
| H04 162616 | 723 / 723 | 498 |
| H05 162834 | 410 / 644 | 44 |

H02 161831 및 H05는 subset GT가 특히 적다. 프레임 유지율만으로 충분한 객체 평가
분량이라고 해석하지 않는다. H05는 원래 training_candidate이므로 독립 test 결론에서 제외한다.

### Bag별 E25

| bag / 시각 | AP % | Track F1 % | IDs | Center p50 m | Speed MAE m/s | 2D AVE m/s | FDE@1/2/3s m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| H01 161315 | 90.93 | 95.21 | 0 | 0.181 | 0.452 | 0.529 | 0.623 / 1.047 / 1.642 |
| H02 161616 | 91.36 | 97.26 | 0 | 0.162 | 0.370 | 0.492 | 0.418 / 0.672 / 0.959 |
| H02 161831 | 82.33 | 89.86 | 0 | 0.451 | 0.586 | 0.607 | 0.818 / 1.404 / 2.118 |
| H02 162108 | 92.72 | 98.64 | 0 | 0.171 | 0.545 | 0.618 | 0.649 / 1.137 / 1.683 |
| H03 162301 | 74.35 | 89.35 | 0 | 0.425 | 0.837 | 0.980 | 0.942 / 1.244 / 1.504 |
| H04 162616 | 94.44 | 98.34 | 0 | 0.163 | 0.527 | 0.607 | 0.699 / 1.520 / 2.584 |
| H05 162834 | 63.53 | 100.00 | 0 | 0.803 | 0.480 | 0.557 | 1.022 / 1.331 / 1.732 |
| R01 163034 | 83.31 | 95.20 | 0 | 0.276 | 0.405 | 0.598 | 0.812 / 2.085 / 4.198 |
| R02 163143 | 84.29 | 97.29 | 0 | 0.301 | 0.341 | 0.563 | 0.701 / 1.521 / 2.816 |
| R03 163306 | 81.44 | 95.17 | 0 | 0.311 | 0.446 | 0.659 | 0.955 / 2.463 / 4.755 |
| P01 163434 | 59.99 | 88.76 | 0 | 0.189 | 0.403 | 0.586 | 1.049 / 2.245 / 3.818 |
| P02 163543 | 65.27 | 92.23 | 1 | 0.177 | 0.300 | 0.433 | 0.862 / 2.147 / 4.170 |
| P02 163648 | 63.68 | 93.91 | 0 | 0.209 | 0.365 | 0.512 | 1.020 / 2.148 / 3.521 |
| O01 163803 (충돌 사례) | 59.39 | 88.05 | 1 | 0.116 | ≈0 | ≈0 | 0.317 / 0.326 / 0.335 |
| O01 163859 | 64.43 | 94.07 | 4 | 0.259 | ≈0 | ≈0 | 0.315 / 0.313 / 0.313 |
| O02 164012 | 64.58 | 91.49 | 2 | 0.223 | ≈0 | ≈0 | 0.344 / 0.346 / 0.349 |

고주로·로터리에서는 근시점 ID switch가 없지만 누락/rebirth가 전혀 없다는 뜻은 아니다.
장애물 ID switch 7회와 보행자 1회는 별도 확인이 필요하다. 충돌한 O01을 제외한 나머지
두 obstacle bag에서도 6회가 남으므로 충돌 녹화만으로 설명되지 않는다.
보행자는 옛 AP 약 99%가 새 수동주행 bag에 재현되지 않았다. 로터리와 보행자 3초 예측은
여전히 약 3.8–3.9m이며, 고주로보다 먼 horizon 오차가 크다.

## P7: 동일 고주로 evaluation_locked 5-bag 비교

두 모델 모두 position-only tracker(mode 0)를 사용한 비교이다. detector velocity를 강제로
tracker에 넣은 결과가 아니다. 같은 raw GT의 미식별 프레임 제외 조건을 사용했다.

| 모델 | AP % | Tracking F1 % | Speed MAE m/s | 2D AVE m/s | FDE@1/2/3s m | ID switch |
| --- | ---: | ---: | ---: | ---: | --- | ---: |
| E25 | 87.04 | 94.69 | 0.573 | 0.661 | 0.705 / 1.195 / 1.769 | 0 |
| Q1-P7 | 86.45 | 94.19 | 0.581 | 0.673 | 0.714 / 1.208 / 1.796 | 0 |

새 locked bag에서 P7은 E25를 이기지 못했다. 작은 차이이고 runtime scheduling도 완전히
bitwise deterministic하지 않으므로 “명백한 퇴행”으로 단정하지 않지만 교체 근거는 없다.

development/H05까지 포함한 고주로 7-bag macro도 P7 AP 83.60%, F1 95.17%,
tracker speed MAE 0.549m/s, AVE 0.641m/s, FDE@3s 1.763m였다.
E25의 84.24% / 95.52% / 0.542 / 0.627 / 1.746 대비 교체 근거가 없다.

P7 detector-only 속도는 같은 locked subset에서 종방향 velocity MAE **0.832m/s**,
벡터 크기 speed MAE **0.909m/s**, 2D AVE **1.355m/s**이다. 세 수치는 서로 다른 지표이다.
Tracker의 0.581/0.673과는 sampling/TP 집합도 다르므로 paired estimator 우열 검정을 뜻하지 않는다.
그럼에도 raw detector 속도 오차가 0.5m/s 이하로 충분히 일반화됐다는 근거는 얻지 못했다.
각 bag의 query-memory diagnostics로 memory 활성 구간을 확인했고, bag 첫 frame은 active=0이었다.
이 숫자가 temporal 기능이 꺼진 모델을 잘못 평가해서 나온 것은 아니다.

Production은 **E25 + position-only tracker**를 유지한다. 새 bag을 보며 파라미터를
튜닝하거나 이 bag을 학습 데이터에 넣지는 않았다. 현재 증거는 기존 개발 bag의 P7 개선을
폐기하는 것이 아니라, 그 개선이 새 분포의 production 우위까지 증명하지 못했다는 뜻이다.

## 기존 평가와 대응

[E25 선정 당시 평가](260918_base3_e40_production_evaluation_results.md)의 AP는
로터리 75.65%, 합류 57.00%, 고주로 순항 77.16%, 보행자 99.99/98.89%,
장애물 60.02/64.72%였다. 차량 3-bag macro는 tracking F1 93.91%, speed MAE
0.5683m/s, 2D AVE 0.7252m/s, top-1 FDE@1/2/3s 1.0849/2.4427/4.4912m였다.

새 고주로 bag은 합류→순항→추월 후보 구간을 한 번에 녹화했다. 옛 합류/순항 두 bag과
완전히 동일한 구간별 평가가 아니므로 각각 직접 대응하는 단일 숫자를 만들지 않는다.
새 로터리·보행자 횡단·scale=1 multi-obstacle은 해당 미션의 추가 평가 자료이다.
옛 표와 새 표에는 bag 분포·GT 보정·tracker revision·map 차이가 있으므로 수치 차이를
모델 개선율로 계산하거나 이전 보고서 숫자를 조용히 덮어쓰지 않는다.

## 산출물과 재현

실행 코드 기준 HEAD `6f3ae8b9878449b25268511272afdf6adb271758` + 이 평가 지원용 working-tree 변경.
Tracker submodule `b03777b10b86ba975b8d8e5c26adb21715c2d5b5`.
실행 tracker binary SHA256 `ef67431d3c12987e5d8fd3e9384332b338db9e86157c2e66bf46d4c157101f5c`.
지도 SHA256 `5111ae16ba788b059e40e88201b51b0cb4d1d662dcd8456f43cda9705066be58`.
평가기 테스트는 ROS 및 pipeline source PYTHONPATH를 설정한 상태에서 43개 통과했다.
보행자 heading과 이동 방향이 수직인 경우의 속도 GT, 미식별 GT 프레임 제외 회귀 테스트를 포함한다.

결과 루트: `artifacts/evaluation/bevfusion/teammate_manual_261004/`.
각 `e25|p7/<run_id>_<시각>/`에는 detector 출력 bag, pipeline 출력 bag, 로그,
GT 이름 매핑, 실행 계약과 JSON summary가 있다. 원본 bag과 산출물은 Git ignore를 유지한다.

```bash
cd ~/AjouNice2026
# 이미 완료된 case는 건너뛴다. Docker 이미지와 두 checkpoint가 로컬에 필요하다.
/usr/bin/python3 perception/detection3d/bevfusion/scripts/run_teammate_manual_evaluation.py

source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
export PYTHONPATH="$PWD/base_ws/src/tracking_prediction_evaluation/src:$PYTHONPATH"
# 저장한 출력 재집계이며 모델 재추론을 하지 않는다.
/usr/bin/python3 perception/detection3d/bevfusion/scripts/reevaluate_teammate_resolved_frames.py
/usr/bin/python3 perception/detection3d/bevfusion/scripts/summarize_teammate_manual_evaluation.py
/usr/bin/python3 perception/detection3d/bevfusion/scripts/summarize_teammate_manual_evaluation.py --resolved
```

자동 표: `tables.md`, `tables_resolved_frames.md`.
정량 상세: `aggregate.json`, `aggregate_resolved_frames.json`과 각 case summary.
`evaluation_locked` 5개 고주로와 H01 development/H05 training_candidate는 별도 집계한다.
이 작업에서는 checkpoint 재학습이나 locked bag을 이용한 파라미터 튜닝을 하지 않았다.
