# 고주로 Tracker·Velocity 개선 실험

> **문서 상태: EVIDENCE.** MORAI 없이 저장된 BEVFusion detections를 production
> EKF tracker와 Map-CV에 다시 넣어 tracker 병목을 분리한 결과다.

## 평가 계약 수정

- Primary bag: `vehicle_highway_merge`, `vehicle_highway1`.
- 전역 회귀 확인: production E25의 `vehicle_roundabout_new`.
- Playback은 1.0x이며 raw bag에서 `/Ego_topic`과 detection topic만 발행한다.
  카메라·LiDAR 전체 playback은 30 Hz ROS-time scheduler를 굶겨 동일 조건에서도
  출력 frame 수가 달라졌으므로 폐기했다.
- Speed MAE는 기존 호환성을 위해 object body-x 속도 오차다.
- AVE는 body-x/body-y twist를 map frame으로 변환한 실제 2D vector L2 오차다.
  종·횡 velocity MAE도 GT heading 기준으로 함께 낸다.

## Position measurement noise

FT-E7 고주로 두 bag과 production E25 로터리 한 bag의 equal-bag macro다.

| Rxy m | F1 ↑ | Center p50 m ↓ | Speed MAE ↓ | 2D AVE ↓ | Long/Lat MAE ↓ | Stable minFDE@3s ↓ | All Top-1 FDE@3s ↓ |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.30 | **0.9362** | **0.458** | 0.579 | 0.836 | 0.571/0.476 | 1.587 | 4.548 |
| 0.35 | 0.9355 | 0.460 | 0.564 | 0.818 | 0.553/0.466 | 1.497 | 4.505 |
| **0.40** | 0.9352 | **0.458** | 0.559 | 0.805 | 0.546/0.457 | 1.446 | **4.492** |
| 0.45 | 0.9345 | 0.464 | **0.557** | **0.799** | **0.544/0.450** | **1.429** | 4.543 |

`0.40 m`는 center를 유지하면서 speed·AVE와 stable/all prediction을 모두 개선하는
균형점이다. `0.45 m`는 velocity와 stable slice는 더 좋지만 전체 Top-1 FDE가 다시
나빠져 채택하지 않았다.

production E25 고주로 두 bag만 재검증해도 `0.30 -> 0.40 m`에서 F1
`0.9324 -> 0.9341`, speed `0.655 -> 0.645 m/s`, stable 3초
`1.766 -> 1.696 m`, all 3초 `4.695 -> 4.663 m`로 개선됐다. center p50은
`0.599 -> 0.615 m`로 1.6 cm 나빠지는 허용된 trade-off다.

## Motion model

FT-E7 고주로 두 bag 결과다.

| Model | F1 ↑ | Center p50 ↓ | Speed MAE ↓ | 2D AVE ↓ | Stable minFDE@3s ↓ | All Top-1 FDE@3s ↓ |
|---|---:|---:|---:|---:|---:|---:|
| CTRV | 0.9330 | 0.477 | **0.668** | 0.891 | 1.788 | 4.695 |
| CV | **0.9335** | 0.477 | 0.669 | **0.846** | 1.808 | 4.717 |
| CA | 0.9310 | **0.471** | 0.727 | 0.952 | 2.516 | **4.493** |
| CTRA | 0.9316 | **0.471** | 0.726 | 0.962 | 2.536 | 4.500 |

CV는 횡 velocity를 안정화하지만 로터리의 전체 3초 FDE를 E25 기준
`4.255 -> 4.555 m`로 악화했다. CA/CTRA는 가감속 전체 slice를 일부 맞추는 대신
현재 속도와 안정 순항을 크게 망쳤다. 따라서 전역 motion model은 CTRV를 유지한다.

## GT velocity oracle

BEVFusion box는 그대로 두고, 과거 0.5초 GT bbox-center 위치만 사용해 causal velocity를
복원했다. 고주로 두 bag에서 매칭 659개 중 657개 detection에 velocity가 주입됐다.
채택된 `Rxy=0.40 m`와 sensor-topic filtered playback 조건의 결과다.

| Velocity 사용 | F1 ↑ | Center p50 ↓ | Speed MAE ↓ | 2D AVE ↓ | Stable minFDE@3s ↓ | All Top-1 FDE@3s ↓ |
|---|---:|---:|---:|---:|---:|---:|
| Init only | 0.9323 | 0.487 | 0.644 | 0.852 | 1.617 | 4.672 |
| Every update, noise 0.1 | 0.9335 | 0.577 | **0.240** | **0.249** | 0.612 | **3.466** |
| **Every update, noise 0.5** | **0.9339** | **0.388** | 0.249 | 0.259 | **0.594** | 3.493 |

좋은 velocity observation을 매 update에 넣으면 position-only tracker보다 speed·AVE와
prediction이 크게 개선된다. 그러나 지나치게 작은 covariance `0.1 m/s`는 causal
position-derived velocity의 frame noise까지 신뢰해 center를 망친다. 실제 velocity
head를 추가할 때도 fusion mode 2와 약 `0.5 m/s`부터 보수적으로 검증한다.

## 최종 결정

1. Production tracker는 CTRV를 유지하고 position measurement noise를 `0.40 m`로
   변경한다.
2. Detector velocity fusion은 기존 7D detector 호환을 위해 기본 `off`를 유지한다.
   launch/runtime A/B 인자만 정식으로 노출했다.
3. Tracker tuning만으로 얻는 이득은 유효하지만 제한적이다. 다음 큰 폭의 motion 개선은
   BEVFusion velocity head 또는 temporal detector가 실제 velocity observation을 내는
   통제 실험이다.
4. Prediction acceleration/Top-1 lane ranking 변경은 tracker 변경과 섞지 않고 다음
   별도 실험으로 진행한다.

## 재현 artifact

- Parameter sweep: `artifacts/evaluation/bevfusion/highway_tracker_ablation_260919/`
- Rxy refinement: `artifacts/evaluation/bevfusion/tracker_rxy_refinement_260919/`
- Production E25 check: `artifacts/evaluation/bevfusion/production_e25_tracker_rxy_crosscheck_260919/`
- Velocity oracle: `artifacts/evaluation/bevfusion/highway_tracker_velocity_oracle_260919/`
