# BEVFusion 문서 index

마지막 정리: 2026-10-05.

## CURRENT

Base5/E25의 [batch1·ROS 포화 처리량](261006_BASE5_BATCH1_CAPACITY_EVIDENCE.md)을 측정했다.
5060Ti에서E25 offline13.90FPS, 원속도ROS11.47Hz/드롭0, 약98Hz stress 입력에서출력12.97Hz다.
320×896/±100m fresh후보는offline10.58FPS cost probe이며 정확도/4090/확대후보ROS 결과는 아니다.
[단계별비용·격자/depth대조](261006_BASE5_LATENCY_STAGE_BREAKDOWN.md)에서 비용상쇄를실측했다.
기존격자+0.5mdepth로100m highres를보는비용대조는E25대비약1.98배다.현재후보의속도를무료정확도개선으로해석하지않는다.

독립 Base5 생성/학습 후보는 [Base5 v2 CURRENT](../../dataset/morai_od/docs/BASE5_V2_CURRENT.md)를 우선한다. 생성된10K는 구왕복 이력이며 최신 일방향 기본의10K는 아직 생성하지 않았다. v25 yaw 실측은42/42조건 완료,Park 재시도 성공이며 간헐timeout 원인은 미해결이다. 100m 직사각형 BEV 및4-class 후보의 단일 GPU loss/역전파 smoke와 production E25를 구분한다.

Tracker 속도 평가의 GT 기준은 [velocity GT 재검토·실행 설정](261005_VEHICLE_VELOCITY_GT_REFERENCE_REVIEW.md)를 함께 읽는다. 보행자 ROS 메시지 우선/이동-zero fallback은 단위·좌표계 명시 opt-in으로 구현했다. 차량은 위치 전체XY/메시지 전체XY/차축→중심 가정 보정의 세 기준을 동일 표본으로 유지한다. H01/R01 및 UDP P01 baseline 재현과 pytest55개를 통과했다. 물리 velocity 기준점과 새 ROS 실주행 A/B는 미확정이며 기존 기본값과 production은 그대로다.

Base5 E25-init E2 학습과 [E1 보행자·장애물9-bag rosbag A/B](261005_BASE5_E1_MISSION_ROSBAG_AB.md)는 완료됐다. 장애물 소폭 개선이지만 초근접·원거리 보행자 공백은 미해결이다. Production은 E25 그대로다. [Base3 공백·학습 결과·후속 보강](261005_BASE3_GAPS_AND_BASE5_PILOT_RESULTS.md)을 함께 읽는다.

| 목적 | 문서 |
|---|---|
| Production E25 실행·판단팀 전달 | [`BASE3_BEVFUSION_PRODUCTION_HANDOFF.md`](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md) |
| 현재 개선 우선순위: 근접·원거리·차량 승격·고해상도 | [`261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md`](261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md) |
| Production E25 rosbag Tracking/Prediction RViz 재생 | [`E25_ROSBAG_RVIZ_RUNBOOK.md`](E25_ROSBAG_RVIZ_RUNBOOK.md) |
| 팀 공용 Docker 학습 | [`TEAM_DOCKER_TRAINING_HANDOFF.md`](TEAM_DOCKER_TRAINING_HANDOFF.md) |
| Base4 temporal 팀원 인계·재현 | [`BASE4_TEMPORAL_TEAM_HANDOFF.md`](BASE4_TEMPORAL_TEAM_HANDOFF.md) |
| Base3-Online dataset 현재 상태 | [`../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`](../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md) |
| 다음 velocity/temporal 통제 실험과 architecture ladder | [`260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`](260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md) |
| Base4 24-scene 수집·9D/3-sweep·학습 smoke 결과 | [`260921_BASE4_TEMPORAL_P24_PIPELINE_SMOKE.md`](260921_BASE4_TEMPORAL_P24_PIPELINE_SMOKE.md) |
| Base4 V1/T1 E4 고주로 detection·tracking·prediction 판정 | [`260921_BASE4_V1_T1_E4_HIGHWAY_EVALUATION.md`](260921_BASE4_V1_T1_E4_HIGHWAY_EVALUATION.md) |
| Base4 velocity head learnability·LR 진단 | [`260922_BASE4_VELOCITY_LEARNABILITY_AND_LR_ABLATION.md`](260922_BASE4_VELOCITY_LEARNABILITY_AND_LR_ABLATION.md) |
| Base4 center-motion/association 오차 분해 | [`260923_BASE4_CENTER_MOTION_PROBE.md`](260923_BASE4_CENTER_MOTION_PROBE.md) |
| Base4 T2 sweep-separated temporal BEV 구현·smoke | [`260922_BASE4_T2_TEMPORAL_BEV_P0.md`](260922_BASE4_T2_TEMPORAL_BEV_P0.md) |

Base4 수집·학습·Q1 실험 코드는 최신 production 기준 위의
`perception/base4-temporal-competition-v2`에 복구했다. 모든 temporal/velocity 옵션은
기본 OFF이며, 이 복구가 production E25를 교체한다는 뜻은 아니다.

Production 표준은 `B3-BF0-F40 E25`, class threshold는 `0.40/0.15/0.10`이다.
Offline 최고 epoch만으로 production을 바꾸지 않고 detection, tracking, Map-CV를 같은
mission rosbag에서 비교한다.

## EVIDENCE

- [`261005_VEHICLE_VELOCITY_GT_REFERENCE_REVIEW.md`](261005_VEHICLE_VELOCITY_GT_REFERENCE_REVIEW.md)
  - 새 H01 E25 동일 tracker 출력의 위치 미분/메시지 속도 GT A/B, 짧은 이력 문제, 단위 이력 정정 및 차축→BBox 중심 속도 기준 미확정 경계.

- [MR VLP16 실제 출력100m 제한](261005_MR_VLP16_100M_RANGE_EVIDENCE.md): fireDistance150 저장 유지에도 원본 UDP/pointcloud는 약100m 이내. 원거리7대와55.6m control 차량 live 대조. 내부 원인/대회 규정 제한을 확정한 것은 아니다.

- [`261005_BASE5_E1_MISSION_ROSBAG_AB.md`](261005_BASE5_E1_MISSION_ROSBAG_AB.md)
  - E1 기존/새 보행자·장애물9-bag 완료, 동일 E25 계약 Detection/Tracking/Map-CV 및 거리별 검출률. E2·차량 미션은 이번 범위 밖.
- [`261005_BASE3_GAPS_AND_BASE5_PILOT_RESULTS.md`](261005_BASE3_GAPS_AND_BASE5_PILOT_RESULTS.md)
  - 실제 Base3 train 거리/point 분포, Base5 E1/E2 완료 결과와 미검증 경계, 다음 고정 계약 A/B·표적 보강 순서.
- [`261004_E25_OLD_NEW_CURRENT_CONTRACT_COMPARISON.md`](261004_E25_OLD_NEW_CURRENT_CONTRACT_COMPARISON.md)
  - 기존 6-bag도 현재 E25/NMS·tracker·GT catalog·v85로 재실행한 기존/새 미션 대응 표. 아래 historical-baseline 직접 비교를 대체한다.
- [`../../dataset/morai_od/docs/261004_base5_reinforcement_smoke_runbook.md`](../../dataset/morai_od/docs/261004_base5_reinforcement_smoke_runbook.md)
  - Base5 근접·실제 고주로 전후방 24 scene + 저속 통과 2 scene, 84-frame 변환과 sparse/ROI 경계 감사.
- [`261004_BASE5_LIVE_ACQUISITION_PROBE.md`](261004_BASE5_LIVE_ACQUISITION_PROBE.md)
  - MR 근접/원거리 7-scene rosbag 수집·21-frame 변환, 동기 캡처 annotation 미완료와 최소 프로필 복원 기록.
- [`261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md`](261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md)
  - 16-bag 근접/전달 감사, opt-in 승격 단위 검증·H03 replay, 5060 Ti 고해상도 GPU smoke.
- [`261004_NEW_VS_EXISTING_ROSBAG_ANALYSIS.md`](261004_NEW_VS_EXISTING_ROSBAG_ANALYSIS.md)
  - 기존 E25 미션 평가와 새 팀원 bag 대응, 거리별 누락·center 정밀도·미래 경로 오차 분석.
- [`261004_VEHICLE_STATUS_SIZE_GT_RECOVERY.md`](261004_VEHICLE_STATUS_SIZE_GT_RECOVERY.md)
  - UDP 차량 기본 제원→geometric BBox 대응 복구와 재평가. 아래 첫 평가의 차량 결과를 대체한다.
- [`261004_TEAMMATE_MANUAL_FULL_PIPELINE_EVALUATION.md`](261004_TEAMMATE_MANUAL_FULL_PIPELINE_EVALUATION.md)
  - 수동주행 E25 16-bag / Q1-P7 고주로 7-bag detection·tracking·Map-CV 평가와 GT 미식별 프레임 감사.
- [`261004_TEAMMATE_MANUAL_ROSBAG_RAW_AUDIT.md`](261004_TEAMMATE_MANUAL_ROSBAG_RAW_AUDIT.md)
  - 팀원 수동주행 16-bag 원본 검증, 고주로 실제 속도 분포 및 정식 평가 전 GT/시간축 조건.
- [`260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md`](260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md)
  - Base3-Online Vehicle-only E25 fine-tuning의 7-bag 및 tracking/Map-CV 결과.
  - Vehicle detection·center·2D AVE는 개선됐지만 Obstacle 망각 때문에 E25 유지.
- [`260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md`](260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md)
  - Tracker motion model/noise, 올바른 2D AVE 계약, GT-velocity oracle 근거.
- [`260921_ROSBAG_ASSET_CATALOG_GT_REEVALUATION.md`](260921_ROSBAG_ASSET_CATALOG_GT_REEVALUATION.md)
  - 학습 변환기와 rosbag 평가기의 asset별 BBox center/size 계약 통합 및 기존 고주로 A/B.
- [`260918_base3_e40_production_evaluation_results.md`](260918_base3_e40_production_evaluation_results.md)
  - E25를 production으로 선택한 offline/rosbag/tracking/Map-CV 근거.
- [`260924_STATIC_OBSTACLE_TRACKING_DIAGNOSIS.md`](260924_STATIC_OBSTACLE_TRACKING_DIAGNOSIS.md)
  - Rotated BEV NMS와 tracker lifecycle/duplicate-birth 개선의 saved-bag 근거.
## HISTORICAL

- [`legacy/BASE3_BF1_ONLINE_DOMAIN_FINETUNE.md`](legacy/BASE3_BF1_ONLINE_DOMAIN_FINETUNE.md): 이전 BF1 image-domain augmentation 실험.
- [`legacy/260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md`](legacy/260919_BASE3_ONLINE_HIGHWAY_TEAM_HANDOFF.md): 완료된 fresh/E25-init 팀 학습 절차.
- [`legacy/260918_base3_e40_post_training_checklist.md`](legacy/260918_base3_e40_post_training_checklist.md): 완료된 E40 사후평가 절차.
- `legacy/260720_morai_four_camera_zero_shot_*`: 초기 zero-shot feasibility 기록.

HISTORICAL 문서의 checkpoint, image tag, dataset root, threshold를 현재 기본값으로
사용하지 않는다.
