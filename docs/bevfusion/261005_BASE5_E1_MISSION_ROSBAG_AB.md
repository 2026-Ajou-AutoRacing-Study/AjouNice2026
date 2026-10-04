# Base5 E1 보행자·장애물 rosbag A/B

상태: EVIDENCE, 2026-10-05. Production E25 유지.

## 먼저 읽을 결론

**E1은 장애물에서 소폭 개선됐지만, 초근접 장애물·원거리 보행자 공백을 극복하지 못했다.**
Offline best가 E1이라는 이유로 production을 바꾸지 않는다. 다음 작업은 epoch 증가보다 실제 실패 거리/각도의 학습 보강이다.

시간 제약에 따라 E2·고주로·로터리 평가를 취소했다. E1만 기존 Pedestrian2/Obstacle1 + 새 Pedestrian3/Obstacle3, 총9 bag을 대상으로 삼았다.
총9 bag 완료, `comparison.json` status=complete / pending=0이다. 평가용 inference Docker도 종료했다.

## 동일 계약

- 후보: `bf-sw-base5-nearfield-replay-e2-pilot-rtx5060ti-261004-214609/epoch_1.pth`.
- baseline: [현재 계약 E25 기존/새 평가](261004_E25_OLD_NEW_CURRENT_CONTRACT_COMPARISON.md). 새 추론 및 tracker/predictor 재생을 후보에 실행했다.
- class threshold 0.40/0.15/0.10, class-aware rotated NMS IoU0.5, single-frame, raw/deskew OFF, detector velocity OFF.
- 같은 installed tracker binary, CTRV/XY noise0.4m/internal age0.5s, experimental promotion/center-motion OFF. 지도v85·status-size v2 catalog 해시가 baseline과 일치한다.
- Bag별 heading 계약을 그대로 사용한다. MR -1.347° 한 번, 과거 보행자 bag은 기존0° 계약 유지. 같은 asset-name map, geometric center/size/center velocity 및 unresolved-frame mask다.
- 추론0.5배속, 저장된 detection의 tracker/predictor 재생1배속. 360°/30m, 후방 제외 없음.
- AP/F1/FDE는 bag 비가중 평균, ID switch는 합계다. 반복 평가한 dev bag이며 새로운 독립 test라고 부르지 않는다.

## 새 bag 전체 결과

| 미션 / 모델 | Center AP | Tracking F1 | ID switch 합계 | Speed MAE | 2D AVE | FDE@1/2/3s |
|---|---:|---:|---:|---:|---:|---|
| Pedestrian3 / E25 | 62.98% | 91.63% | 1 | 0.356m/s | 0.510m/s | 0.977 / 2.180 / 3.836m |
| Pedestrian3 / E1 | 61.94% | 91.96% | 0 | 0.359m/s | 0.524m/s | 0.986 / 2.201 / 3.882m |
| Obstacle3 / E25 | 62.80% | 91.20% | 7 | ≈0 | ≈0 | 0.325 / 0.328 / 0.332m |
| Obstacle3 / E1 | 64.00% | 92.07% | 4 | ≈0 | ≈0 | 0.328 / 0.331 / 0.334m |

보행자는 AP -1.04%p, F1 +0.33%p지만 속도/미래경로는 소폭 악화했다. 장애물은 AP +1.20%p, F1 +0.87%p, ID switch -3회다.
장애물 FDE는 거의 같고 matched 객체 조건부 값이다. 검출이 안 된 객체까지 안전하게 회피한다는 뜻이 아니다. 속도≈0은 정적 객체 정책의 결과다.

기존 bag 회귀 결과: Pedestrian2의 AP99.27%→98.49%, F199.71%→99.74%, FDE@3s0.632→0.644m다. Obstacle1은 AP68.85%→72.45%, F193.96%→94.29%, ID switch4→4회, FDE@3s0.345→0.356m다. 기존 장애물 AP 상승도 matched center/미래경로 정확도 개선까지 뜻하지는 않는다.

## 핵심: 실제 실패 거리의 변화

Production score를 적용한 class별 1m confidence-order matching을 전체30m에서 먼저 수행하고, match된 GT를 Ego 거리 bin에 귀속했다. 거리 bin에서 별도 재매칭하지 않는다. 반복 관측 GT 수를 합친 pooled recall이며 bag macro AP와 다른 집계다.

| 새 bag 구간 | GT 수 | E25 TP / recall | E1 TP / recall |
|---|---:|---:|---:|
| Pedestrian 20–30m | 88 | 15 / 17.05% | 14 / 15.91% |
| Obstacle 0–2m | 20 | 0 / 0% | 0 / 0% |
| Obstacle 2–5m | 380 | 120 / 31.58% | 129 / 33.95% |
| Obstacle 5–10m | 349 | 344 / 98.57% | 345 / 98.85% |
| Obstacle 10–20m | 743 | 716 / 96.37% | 724 / 97.44% |
| Obstacle 20–30m | 775 | 450 / 58.06% | 459 / 59.23% |

Obstacle 2–5m의 추가 TP는 9개뿐이다. asset별로 CargoBox +1 / TrafficBarrel +4 / concrete-barrier +2 / Concrete_Barricade01 +2, PE_Drum·PE_Firewall_Orange 변화0이다.
특히 긴 Concrete_Barricade01은 152개 GT 중 TP16→18, CargoBox는49개 중6→7로 여전히 약하다.
0–2m20개는 적고 반복 관측을 포함하므로 모든 상황의 절대 한계를 증명하지는 않는다. 다만 **이번 평가에서 해결되지는 않았다**는 직접 근거다.

## 왜 개선 폭이 작았나

- 확인된 사실: 최신 복구 addon은 정지8/12m ×yaw0/90°다. 0–2m 실제 옆면 통과·부분 시야나20–30m 보행자를 직접 보강한 데이터가 아니다.
- 확인된 사실: 새 장애물 평가에는 CargoBox/PE_Drum/TrafficBarrel/concrete-barrier/Concrete_Barricade01/PE_Firewall_Orange가 있다. 복구 addon의 WoodBox/police_f_1/Scooter_01 성능을 이 표로 판정할 수 없다. 특히 WoodBox GT는 평가에 없다.
- 확인된 사실: 소량727-frame train의 E25-init E1, LR2e-5 학습이다. 전체 대표성을 확보한 대규모 재학습은 아니다.
- 해석: 필요한 appearance 공백을 충분히 채우지 않은 소량 적응이라 일반 미션 변화가 작다는 설명과 맞는다. '더 오래 학습하면 해결' 또는 'rosbag 수집 자체가 무효'라는 결론은 근거가 없다.
- CargoBox 새 MR geometric reference와 역사적 reference의 차이는 별도 미해결이다. 그 차이가 이번 지표에 미친 양을 분리 검증하지 않았다.

## 다음 실행

1. Production E25 유지. 이번 E1의 차량 미션은 미평가이므로 전역 교체하지 않는다.
2. Scale1로 긴 바리게이트/CargoBox 등 실제 실패 asset의 근접0.5–5m 통과를 좌우·yaw·부분 시야별로 보강한다. 정지8/12m scene만 추가하지 않는다.
3. Pedestrian20/25/30m를 다양한 인물·방향으로 보강하고 camera-visible sparse GT만 sample+UID로 검토한다. 5-point를 전역 제거하지 않는다.
4. 위 보강 + Base3 replay로 짧게 재학습하고 **동일 rosbag 거리별 recall부터** 비교한다. AP만 높아져도 초근접이 그대로면 해결 판정하지 않는다.
5. 고해상도320×896은 동일 데이터·GT·ROI 조건의 별도 A/B다. 장거리 차량30m+는 ROI/학습 범위 확장과 분리한다.

## 재현·산출물

- 실행: `python3 perception/detection3d/bevfusion/scripts/run_base5_checkpoint_rosbag_ab.py --models e1 --missions pedestrian obstacle`.
- 집계: `python3 perception/detection3d/bevfusion/scripts/summarize_base5_rosbag_ab.py`.
- 세부 recall: ROS workspace를 source하고 평가 library를 PYTHONPATH에 추가한 뒤 `audit_base5_ab_distance_recall.py` 실행.
- root: `artifacts/evaluation/bevfusion/base5_nearfield_replay_e2_261005/`.
- `runtime_manifest.json`: 모델·원본·catalog/tracker/map 해시, 평가 범위.
- `comparison.json`, `comparison.md`: 완료 여부/개별 bag/mission macro.
- `distance_recall.json`: 거리·asset별 GT/TP/FN.
- `e1/*/detections.bag`, `pipeline/pipeline_outputs.bag`, 각종 runner log와 `_status_size_v2` summary.
- 기존 E25 산출물은 보존하고 baseline 경로로 연결한다. 원본 bag/dataset은 Git ignore 계약 유지.
