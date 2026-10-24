# Localization 평가 입력 계약

상태: CURRENT. 2026-10-24에는 목록을 정리했으며 원본 bag/snapshot/events는 변경하지 않았다.
전체 실행·부정 결과·정확한 bag 경로는
[원본 catalog](legacy/evidence/260924_LOCALIZATION_EVALUATION_BAG_CATALOG.md)에 보존한다.

## 등급과 oracle

- `REPLAY`: raw estimator 입력으로 같은 입력을 재실행할 수 있다.
- `CLOSED_LOOP`: 실제 차량 feedback의 localization/planning/control 결과가 있다.
- `FIELD_RESULT_ONLY`: 현장 출력은 있으나 raw 입력 부재로 estimator 재실행은 불가하다.
- `DIAGNOSTIC_ONLY`: 시간축·부하·route 오염으로 production 채택 근거가 아니다.

MR/Competition 실행의 raw Ego oracle heading offset은 `-1.347 deg`, MRP replay와
보정 없는 직접 GT는 `0 deg`다. 파일명·취득 PC로 추측하지 않는다.
`analyze_localization_stability_bag.py` 기본값은 MR용이므로 MRP에는
`--heading-offset-deg 0`을 명시한다. estimator 출력에 offset을 다시 적용하지 않는다.

## 우선 입력

| 입력 | 등급·용도 | heading·제한 |
|---|---|---|
| `rosbags/raw_sensor_w_cam/competition_full_run/260822_005713_competition_full_route_raw_hz_run01.bag` | REPLAY; 99초 동일 raw input A/B | MR `-1.347deg` |
| `rosbags/raw_sensor/gps_blank1.bag`, `gps_blank2.bag` | REPLAY; 구형 zero-GNSS 연속성·재획득 보조 | replay `0deg`; 취득 alias 불명, 절대 정확도 채택용 아님 |
| `rosbags/eval/localization/local_mr_stability/260919_231906_q0005_r010_gt_object_lightweight_no_traffic.bag` | REPLAY+CLOSED_LOOP; 과거 경량 heldout | MR `-1.347deg`; 마지막 음영 정차, 복귀 없음 |
| `rosbags/eval/localization/local_mr_stability/260919_233559_sigma0p05_qxy0p05_gt_object_lightweight_no_traffic.bag` | NEGATIVE_CONTROL; 기각 Q/R 회귀 | MR `-1.347deg` |
| `rosbags/eval/localization/260915_field_inspection/planning_rviz_0915_sensor_local_1.bag`, `_2.bag`, `_3.bag` | FIELD_RESULT_ONLY; LAN 출력·epoch 역산 | MR `-1.347deg`; 현재 production LAN 검증으로 승격 금지 |
| 같은 폴더 `planning_rviz_0915_gt_local.bag` | FIELD_RESULT_ONLY; 직접 GT 기준 | `0deg`; MR 기본 offset이면 가짜1.347deg 오차 |

현행 soft/Smooth의 실제 폐루프 근거는 `planner_logs/validation/`의 다음 execution이다.
각 `success/<case>/planbag_*.bag`와 원본 snapshot/events/receipt를 함께 선택한다.

- `loc_fullcourse_soft_production_261015_044421`, `_045214`: 무객체 전체 코스2회.
- `loc_agent_soft_lateral_pair_261015_045709`, `_050237`: 같은 쪽 회피의 음영 pair2회.
- `loc_final_shadow_hold_261015_062342`: 음영 중30초 Park hold·재출발.
- `loc_fullcourse_true_lightweight_261015_112304`: 실제 camera/LiDAR0개 full-course.

모두 MR/Competition·estimator/oracle 각각−1.347°·receipt-time이며 실제 ROS1
planning/PP·PI/od_gt 결과다. 실행 사이 차이는 같은 입력 인과 A/B가 아니다.
Smooth-only 출력 재구성과 estimator replay는 새 차량 폐루프로 세지 않는다.
Spawn 툴 관측은 CP14 음영 조정 전 환경이며 원본 CP13/15 구번호를 보존한다.

## 새 실험 필수 provenance·토픽

bag path, acquisition(MR/MRP/LAN_FIELD/OFFLINE_REPLAY), alias, `gt_source`,
estimator heading correction, evaluator heading offset, time policy, real-time factor,
localization/controller revision, 바꾼 변수·고정 변수, sensor/scenario hash를 기록한다.
복구 불가능하면 절대 yaw·횡/종 오차의 채택 근거에서 제외한다.

최소 raw `/Competition_topic`, `/competition/raw_twist`, `/sensor/gps`, `/sensor/imu`,
개발 oracle `/Ego_topic`, GT orientation/twist, normalized IMU, GNSS odom,
Global/Local odom, Global/Smooth state, soft ON이면 lateral prior, `/tf`, `/tf_static`,
기록된 trajectory·controller debug·steering·`/ctrl_cmd`를 함께 남긴다.
녹화 요청만이 아니라 실제 topic count를 audit한다. Localization-only에는 camera/LiDAR를
기록하지 않는다. [EXPERIMENT_TEMPLATE](EXPERIMENT_TEMPLATE.md)을 따른다.
