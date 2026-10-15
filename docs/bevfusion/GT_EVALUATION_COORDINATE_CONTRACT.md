# GT 평가와 운영 localization 좌표 계약

확인 기준: 2026-10-14. 운영 출력, 독립 평가 GT, 시험용 GT pose를 구분한다.

## 목적에 따른 pose 선택

| 목적 | 사용하는 pose | heading 보정 |
|---|---|---|
| 실제 tracker·planner 실행 및 그 결과 재현 | 기록된 `/localization/kinematic_state`와 해당 TF | 해당 localization에서 이미 적용한 보정을 다시 더하지 않는다 |
| 검출 위치를 simulator 정답과 독립적으로 비교 | 원시 `/Ego_topic` 및 객체 GT를 평가 좌표 계약으로 변환 | 이번 MR 시험은 raw heading에 −1.347°를 한 번 적용한다 |
| localization 자체의 위치·방향 오차 평가 | 원시 Ego에서 별도로 만든 simulator GT | 추정 localization을 정답으로 사용하지 않는다 |
| GT pose를 사용한 detector·tracker 분리 시험 | 보정한 raw Ego로 만든 별도 시험 TF | 실제 운영 localization을 사용한 전체 파이프라인 평가와 구분한다 |

운영 localization을 정답에도 사용하면 localization 오차가 양쪽에 함께 들어가 상쇄될 수 있다.
반대로 실제 tracker 출력은 그대로 보존하고 독립 GT와 비교해야 localization을 포함한 오차를 볼 수 있다.
검출 박스를 GT pose로 재투영한 결과는 검출기 분리 평가이지 운영 전체의 평가가 아니다.

## −1.347°는 어디에 적용하는가

현재 MR 보정 계약의 식은 `corrected_yaw = raw_heading − 1.347°`다.
운영 adapter와 독립 평가기가 각각 **원시 heading에서 출발**하여 이 식을 한 번 쓰는 것은 중복 보정이 아니다.
이미 보정한 localization yaw에서 다시 1.347°를 빼는 것이 중복 보정이다.

운영 설정은 `configs/localization/localization_manager.yaml`의
`ground_truth.use_heading_correction`과 `heading_yaw_offset_deg`를 확인한다.
실제 launch 인자와 bag의 저장 파라미터가 최종 기준이다. generic launch의 기본값만으로 보정 여부를 추정하지 않는다.
평가 도구 `scripts/evaluation/experiment_live_perception_candidates.py`는 별도의
`--heading-correction-deg`를 사용한다. 모든 평가 도구가 공통 설정을 자동으로 읽도록 통합된 상태는 아니다.

MR/MRP 구분은 물리 PC나 ROS/UDP 여부가 아니라 simulator·launch·GT 입력 계약으로 한다.
MRP 및 과거 FIELD-GT의 offset 0 계약에 이번 MR의 값을 일괄 적용하지 않는다.
과거 bag은 [localization 현재 기준](../localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md)과
[bag 목록](../localization/260924_LOCALIZATION_EVALUATION_BAG_CATALOG.md)의 provenance를 먼저 확인한다.

각 평가 결과에는 다음을 기록한다.

- simulator 종류, raw Ego/Object 토픽, pose source와 frame.
- estimator와 evaluator 각각의 보정 활성화 여부와 offset.
- LiDAR 기준 평가 시각, pose 보간·허용 시간차·시간축 정책.
- 시나리오 yaw와 정지 heading 대조 여부, 모델·입력·출력의 보존 범위.

## 이번 시험에서 바로잡은 부분

이번 MR 녹화는 raw heading이 저장한 시나리오 yaw보다 약 1.347° 컸다.
독립 평가 경로의 correction 0을 −1.347°로 바꾸자 정지 yaw 잔차가 0.02° 이내였다.
이미 알려진 localization 보정을 시험용 GT 경로에서 빠뜨린 것이며 새로운 보정 현상을 발견한 것이 아니다.
이 각도 차이만으로도 30m에서 약 0.71m, 100m에서 약 2.35m의 횡방향 비교 오차가 생길 수 있다.

3,647개 token의 LiDAR stamp·점 payload SHA·JPEG SHA는 보정 전후 동일하다.
원본 입력·가중치·이미 계산한 추론 출력은 보존하고, 독립 GT 및 그에 따른 평가 좌표만 다시 계산했다.
production localization 변경, 재학습 또는 재수집을 요구하는 수정이 아니다.
[실측 근거](261014_LIVE_PERCEPTION_CANDIDATE_REVIEW.md#물리-yaw와-기록된-기본-gt-tf의-구분)를 따른다.

이 계약은 BBox bottom/gravity 원점 변환과 별개다.
ROS 출력 중심은 [BBox 원점 계약](261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#box-origin)을 따르고,
자산 지면 높이·부유 여부도 heading 보정으로 해결되지 않는다.
