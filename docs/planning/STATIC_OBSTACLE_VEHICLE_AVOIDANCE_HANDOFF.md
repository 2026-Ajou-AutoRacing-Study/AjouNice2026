# 정지 차량 회피 누락 — Planning handoff

상태: **OPEN / 충돌 재현됨.** 2026-10-02, `localization/competition-stability-v3`
(`945d4fd4`) 기준. 이 문서는 재현 계약과 조사 방향을 전달한다. 개별 bag 분석은
Git 제외 경로 `planner_logs/analysis/261002_hidden_avoidance_ab/README.md`에 둔다.

## 팀장님께 먼저 전달할 결론

같은 GPS 음영 구간에서 차량 2대·3대 배치를 `od_gt → tracking/Map-CV → Planning`
경량 스택으로 주행했다. MORAI `/Object_topic`과 Planning의 예측 객체에는 충돌 전
차량이 있었지만, 정적 장애물 회피 제약에는 일부 차량만 들어갔다. 제외된 차량과
실제 `/CollisionData` 충돌이 발생했다. 세단 한 차종만의 문제가 아니라 트럭도
같은 양상을 보였다. 별도 ROS master에서 **같은 bag과 현재 Planning 코드**를
재생해도 제외 사유 `IS_NOT_PARKING_OBJECT`가 재현됐다.

이 결과는 지난번 `DrivableAreaInfo::Obstacle`의 ABI 불일치로 BPP가 SIGSEGV를
낸 문제와 **별개**다. 현재 PC는 Planning 생성물을 clean 재빌드했고 BPP가 충돌
전후에도 경로를 계속 발행한다. 재빌드 검증 범위는
[BUILD_ABI_CONSISTENCY.md](BUILD_ABI_CONSISTENCY.md)를 참조한다.

## 재현 자료와 조건

- Bag 원본: `rosbags/eval/localization/261002_hidden_avoidance_ab/`와 이전 기준선
  `rosbags/eval/localization/261002_hidden_s2_clean_rebuild/`.
- 시나리오 원본: `configs/scenario/hidden_mission/static_2/` 및 `static_3/`.
  MORAI SaveFile에는 같은 JSON을 `0_hidden_*.json` 이름으로 복사했다.
- 맵: `kcity_map/lanelet_map/lanelet2_map_v85.osm`; Route B.
- 입력: `od_gt`의 `/Object_topic` 기반 detection, tracker와 Map-CV. BEVFusion,
  카메라·LiDAR, 신호등 노드는 끈 경량 시험이다. 따라서 본 건을 BEVFusion
  검출 실패로 해석하면 안 된다.
- 현재 런타임 SOA 주요값: `path_generation_method=optimization_base`,
  `avoidance_for_parking_violation_vehicle.policy=auto`,
  `force_avoid_stopped_vehicle_in_intersection=true`,
  `avoidance_for_close_vehicle.policy=ignore`.
- 현재 브랜치와 `origin/planning/autoware-universe-to-ros1`은 조사 시점에 같은
  commit `945d4fd4`였다. **팀장님 PC의 실제 checkout·설치 바이너리·파라미터가
  같았는지는 확인하지 않았다.**

최소 재현은 `s2_02_truck_sedan_gap10.json`이다. 트럭 ID 100은 회피 제약에
들어가지만 뒤 세단 ID 101은 접근 후 `IS_NOT_PARKING_OBJECT`로 제외되고
충돌한다. `s3_01_three_vehicles_gap10.json`에서는 가운데 차량 ID 101만
제약에 들어가고 뒤 트럭 ID 102와 충돌한다. `s2_13_sedan_barrier_stagger12.json`
에서는 세단은 제외되고 바리케이드는 제약에 들어간다. 이 대조로 자산명이나
한 가지 차량 모델만의 문제보다 **차량 위치·차선/주차 판정 및 후단 정지 안전망**을
우선 조사해야 한다.

## Planning에서 확인할 것

1. `filterTargetObjects()` → `isSatisfiedWithVehicleCondition()` →
   `isNeverAvoidanceTarget()` 경로에서 제외 차량이 왜 `IS_NOT_PARKING_OBJECT`가
   되는지 확인한다. 이 표기는 여러 차선·인접 차선·정지 요인 분기에서 공통으로
   사용하므로, **어느 하위 조건인지는 아직 확정하지 않았다.**
2. 정적 회피 대상에서 제외된 **경로상 정지 차량**을 어느 모듈이 정지 대상으로
   처리해야 하는지 확인한다. 이번 기록에서는 최종 `/planning/trajectory`가
   충돌을 막지 못했다. SOA 필터를 느슨하게 하는 것만으로 안전성이 검증됐다고
   보지 말고 BPP/BVP/MVP/최종 경로 안전검사를 함께 확인한다.
3. 팀장님 PC와 동일 bag을 재생해 `static_obstacle_scene`, SOA의
   `debug/static_obstacle_avoidance` 텍스트, 최종 trajectory를 비교한다.
   동일 bag에서도 결과가 다르면 설치 `.so` SHA, map SHA, runtime `rosparam`
   dump, launch 인자를 비교한다. 결과가 같다면 서로 시험한 시나리오·배치·
   Route가 달랐을 가능성이 크다.
4. 회피 성공 판정은 대상 선정만이 아니라 충돌 0, 최소 이격거리, 유효한 연속
   trajectory와 제어기 폐루프까지 요구한다. 노드 생존이나 Route `VERIFIED`는
   안전 통과 증거가 아니다.

참고: `/Object_topic`의 UDP `name`은 자산명이 아니라 차량의 `link_id`이므로,
정밀한 LiDAR-instance GT 크기 비교는 시나리오의 `UNIQUEID → DataID`와 Base3
asset catalog를 사용해야 한다. 하지만 이것은 **평가 박스 보정 문제**이며,
충돌 전 객체가 Planning 입력에 있었다는 사실이나 SOA의 필터 사유를 바꾸지 않는다.

이 문서는 원인 규명과 수정 승인 전 handoff다. Planning 소스/파라미터를 이
문서 작성 과정에서 변경하지 않았다.
