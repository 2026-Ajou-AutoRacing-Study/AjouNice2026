# Vehicle-only prediction corridor: 2026-10-27 검증 기록

목적: 고주로의 저점수 obstacle/UNKNOWN phantom 때문에 planner가 멈칫한 사례를 줄이되, 검출·tracker 원본을 훼손하지 않는다. 현재 production `tracking` launch는 **판단행 PredictedObjects만** 지정 구간에서 vehicle class로 제한한다. 로터리·고주로 밖은 원형으로 돌아오며 launch 인자로 즉시 OFF 가능하다.

## 경계 선정

- v88/v94 Route A/B 공통 lanelet 경로를 지도 SHA와 함께 확인했다. ON = lanelet 200258의 30m 지점, map `(70.97, 587.42)`; OFF = lanelet 200088 종단, map `(202.10, 160.49)`.
- 샘플의 C 직후 동적 보행자는 map `(74.8, 534.3)`에 있다. C선에서 곧바로 ON하면 이를 가리므로, ON을 약 53m 뒤로 뒀다.
- 검토한 GPS 음영 24개 장면의 가장 가까운 배치 객체는 map `(152.2, 53.5)`이며, OFF 지점과 직선 약 118m다. 이는 **해당 예제 집합의 수치**이며 공식 출제 배치의 보증은 아니다.
- v94 팀장님 bag의 고주로 phantom 제동 부근 map `(202, 168~171)`은 ON 안에 포함된다. 전체 지도는 `python3 scripts/evaluation/render_vehicle_only_corridor_map.py --map-version 94 --output /tmp/vehicle_only_corridor.png`로 재생성한다.

## 검증과 한계

- 2026-10-26 v94 풀주행 bag `rosbags/traffic_light/261026_teamlead_v94_222146/planning_rviz_20261026_222146.bag`을 `scripts/evaluation/audit_vehicle_only_corridor_bag.py`로 읽기 전용 검사했다. ON `+108.6s, (70.95, 587.5)`, OFF `+191.26s, (202.04, 160.86)`로 **한 번씩만** 전환했다.
- 그 bag의 ON 구간에 담긴 예측 객체 occurrence는 vehicle **4,695**, UNKNOWN **13**, pedestrian **70**이다. 새 정책은 이 83개 비차량 occurrence를 planner 출력에서 제거하고 vehicle은 그대로 둔다. 70개 pedestrian 중 C 동적 객체의 잔여 관측도 있으므로, 83개가 모두 오검출이라는 주장은 하지 않는다. 검출·tracking topic은 변경하지 않는다.
- 새 corridor·출력 필터·stale ego fail-open 단위 테스트, v88/v94 지도·launch 기본값 및 v94 노드 startup은 통과했다. 위 팀장님 저장 bag의 토글은 **읽기 전용 후처리**다.
- 로컬 MORAI·validator 실제 planning/PP·PI 폐루프(맵 GT 측위 + `od_gt` 탐지 대조군)에서 독립 객체 배치 고주로 장면 `planner_logs/validation/vehicle_only_corridor_highway_261027_035937`을 47.13초, 충돌 0, 목적지 오차 0.111m로 PASS했다. ON에서 upstream track UNKNOWN occurrence 2,103 / vehicle 15,559, planner prediction UNKNOWN 0 / vehicle 15,374였다. 지도 약 `(201.9,160.8)`에서 OFF 뒤 UNKNOWN이 upstream 159 / prediction 159로 다시 나왔다. 반복 occurrence는 독립 객체 개수가 아니다. 배치한 두 TrafficBarrel은 차로 바깥 실험용이며 공식 미션 배치를 재현하지 않는다.
- 별도 full-course `planner_logs/validation/vehicle_only_corridor_trial_261027_040334`에서는 C의 동적 보행자 위치를 지난 후 지도 `(71.12,588.09)`에서 실제 OFF→ON 전환을 확인했다. 이후 차량 ID 40과 map `(198.3,628.9)`에서 충돌해 전체 완주는 **FAIL**이다. 충돌 0.5초 전 필터된 비차량 track은 ego에서 각각 약 168m 뒤 보행자, 261m 뒤 UNKNOWN이었다. 충돌과 필터의 인과는 검증되지 않았으며, 이 주행을 폐루프 안전성 인증으로 쓰지 않는다. 첫 full-course localization 원형 시험은 시작 후 map `(67.7,176.0)`에서 반복 Invalid Trajectory로 중단했고, 동적구간 단축 시험도 경로 이탈로 중단했다. 따라서 실제 측위+실검출+LAN 풀스택 인증은 남아 있다.
- 매칭 경로·지도 SHA·신선한 `map` ego pose가 없거나 횡거리/방향 조건을 벗어나면 fail-open이다. 따라서 localization 중단 시 phantom을 다시 보낼 수 있고, 반대로 실제 보행자/장애물이 ON 구간에 나오면 planner가 이 prediction을 못 본다. 그 위험을 감수하는 정책이며 공식 배치가 확정되면 다시 판단한다.

운영값과 복귀 방법은 [Tracking CURRENT](../../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)를 따른다.
