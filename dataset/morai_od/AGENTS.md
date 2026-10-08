# MORAI perception dataset agent context

이 디렉터리의 수집·변환 작업 전 다음을 읽는다.

1. `docs/README.md`
2. `docs/BASE3_ONLINE_CURRENT_STATE.md`
   - MR 신규 수집은 `docs/MR_ASSET_COLLECTION_CURRENT.md`의 catalog/profile이 우선한다. 과거 MRP 교집합·quarantine을 MR에 적용하지 않는다.
3. Base3-Online rosbag 수집이면 `docs/base3_online_rosbag_collection_runbook.md`
4. Base4 temporal 작업이면 `docs/BASE4_ROSBAG_CAPTURE_CURRENT_STATE.md`와
   `docs/260920_base4_deskew_and_control_safety_evidence.md`
5. Base5 독립 single-frame 수집이면 `docs/BASE5_V2_CURRENT.md` →
   `docs/261007_base5_direct_5k_current_plan.md` →
   `docs/261007_base5_5k_capture_runbook.md` →
   `docs/261007_base5_final_gates_v105_evidence.md`를 읽는다.
   Sunny13시 직접5K가 현재 목표다. v45/10K 명령·과거 ETA를 재사용하지 않는다.
   `docs/261007_base5_sunny13_environment_correction.md`의 정정이 11시 오표기보다 우선한다.
   진행 중인 수집의 manifest/계약은 덮어쓰지 않는다. 기존 sunny11 config는 13시 내용의 호환 alias다.
   v111부터 정적 미션에도122종 전체를 정지 배치한다. family와 GT class를 혼동하지 않는다.
   v112 동적 미션250셀 중12개는 차량 정지 외관 관측이다. 새manifest 정적4171/이동829,
   구4160limit/구output을 재사용하지 않는다. 차량 양보만8–20m이며 희귀 quota는 유지한다.
   생성기 코드 변경 후 구 output resume 금지. 원래 revision 또는 새 manifest/output을 사용한다.
   v113 현재 manifest는 `final_5k_final_xy_ground_v113_final`이다. 최종 XY 도로 선분 높이와
   자산 pivot을 사용하지만 roadside/unsupported 접지 인증은 아니다. QA 재촬영/대체 상한,
   동일 payload witness1회·단일 master live lock을 유지한다. 구 v112는 `c67f62c3` 재현용이다.
   본수집은 사용자 승인 후이며 raw GT·실패 근거를 보존한다.
   v114는 Ego 최종 footprint 도로 지지·회피 쌍 공동 이동·실제 정지 witness 계약이다.
   구 v113 동결 manifest/output은 보존하되 현재 코드로 본수집/resume하지 않는다.
   v113 재현 revision은 `10578aa8`이다. 새 v114 5K manifest는 별도 준비해야 한다.
   MGeo roadside 추정을 Ego 도로 지지로 인증하거나 Park ACK만으로 정지를 인증하지 않는다.
   v115가 정적 QA 실행 기준이다. `static-single-capture-advisory-v115` 명시+전actor static/standing이면
   원래 LiDAR만 검사한다. plane 부족/중간 높이 오차는 warning, 신뢰 plane 대비±0.40m 초과는 review다.
   정적 높이 확인용 empty/near witness·height recapture를 추가하지 않는다. raw/GT는 수정하지 않는다.
   이동actor 또는 calibration_only는 기존 엄격한 경로 유지, Ego 정지/CollisionData/가시성/GT는 유지한다.
   40셀39승인271.322초→정적4171장8.115h 조건부 외삽; 이동829 포함 전체5K ETA가 아니다.
   구v113/v114 output resume 금지. v115 새전체5K manifest는 아직 준비 전이다.
   2026-10-08 다음은 `docs/261008_base5_intersection_manual_ground_plan.md`다.
   이동 외관은 우선 사거리에서만 취득하는 설계 방향이며 음영 저격은 보류다.
   수동10개/49waypoint·sidecar는 archive 후 MR에서 exact 파일만 정리했다.
   v116 bounded lookup28종×두 구역56profile/132회 PASS다. actual XY 기반 높이를 보간하며
   standalone5K는4846정적/154이동·122종 primary 최소4다. 기존4171/829는 이력이다.
   새release require_covered_moving_paths=true는 경로 밖을 같은 quota로 초기 재생성한다.
   all-editor movers만 terrain/static-reference 준비를 생략한다. 원본 검사/심한±40cm gap·Z step/
   실제 이동·CollisionData·Ego 정지는 유지한다. 접지/지면 mesh/모든23종 외관 차이 인증은 아니다.
   v116 당시 첫 횡단보도 사람 경로 끝이 짧아 running 방향 편향이 남았으며 다음 v117이 우선한다.
   v117은 endpoint-linear-v117 opt-in으로 정확히 두 waypoint/선형 Z를 쓴다. 자동 지면 추종이 아니다.
   짧은 사람 끝점은 같은 구역의 저장 donor 높이와 overlap offset 후보로 보완하고 각각 새 live QA를 했다.
   28종×두 구역×양방향112개 승인용 캡처 PASS, 전체127시도/1LiDAR annotation review는 보존한다.
   실제 이동 primary의 crossing×walk/run×subfamily 방향 차이≤1, same-quota replacement에서도 sign 유지다.
   v117 manifest는 endpoint_linear_final_5k_v117이며4846정적/154이동·122종 최소4를 유지했다.
   v116 raw/manifest는 이력으로 보존하고 새 코드로 기존 output을 resume하지 않는다.
   `docs/261008_base5_intersection_ground_evidence.md`의 최신 batch/resume/경로를 따른다.
   새bulk output에만 새정책을 적용하며 본수집은 보고 후 사용자 승인이다.
   v119 현재 release는 roundabout_release_5k_v119이다. 로터리 미션250셀 geometry를 실제 경로 관측으로 교체한다.
   접근20/대기30/진입60/원형 주행90/탈출50, NPC는 샘플spawn4 시작26→삭제9 범위의5개link만 쓴다.
   원형 주행 앞뒤 NPC 두 대를 raw positive로 검사하며 진입도 두 대 가시성 조건을 적용한다.
   누락은 bounded same-quota replacement/부족분이고 GT를 임의 채우지 않는다.
   NPC 임의XY/yaw jitter 없음; Ego±0.30m/±6°는 최종 도로 지지·실제 정지검사를 유지한다.
   나머지4750 geometry·이동v117/높이v115·122종 최소4는 보존한다. 본선 속도/간격/차종은 하드코딩하지 않는다.
   static_vehicle_policy=park-static-snapshot-v119가 정적 vehicleList NPC의 세 저장 gear를 Park로 맞춘다.
   나머지2314셀 control-only 갱신/2436셀 참조 보존, 이동 actor의 상태는 변경하지 않는다.
   실제 pivot XY25cm 초과 이동/정보 누락, 원형 앞뒤 GT gap quota±30cm 초과는 review다.
   v118 초기47승인 중3셀은 후속 정지 검사 FAIL로 보존하고 최종 검증본으로 쓰지 않는다.
   `docs/261008_base5_roundabout_route_mission_evidence.md`를 읽고 런북 v119 새 경로를 사용한다.
   구v117/v118개발 output에 새runtime resume 금지, 기본config만으로 현재release가 재생성되지는 않는다.
   v121 현재 release는 shadow_release_5k_v121이다. `docs/261008_base5_shadow_observation_evidence.md`가 우선한다.
   위 사거리 이동 한정/음영 보류는 당시 이력이다. 기존 범용500셀을 음영 입구120/내부260/출구120으로
   재배정하며 전체5K/122종 최소4·음영122종 최소1을 유지한다. 기존1750미션/근접/로터리 범용과4500entry 불변이다.
   전체4822정적/178이동(사거리154+음영24)이다. 음영의 static476에는 추가지면촬영이 없다.
   터널 movers는 사거리lookup 인증을 빌리지 않고 independent terrain/static-reference/10–12m witness를 쓴다.
   부족한 경로만 departure16m/12m 상보witness를 조건부추가한다. 전체근거가 확인되면 즉시멈추고8near 상한 유지다.
   AI 이동 미검증은 standing, rigid 정적 자산은 정지, 차량은 Park 외관이며 실제 traffic rollout이 아니다.
   원본GT/ROI/가시성·CollisionData·Park/실제정지·Z gate를 유지한다. 실패/부족분/oldraw는 보존한다.
   본5K는 사용자 승인 후 새output으로 시작하며 개발 pilot에 새runtime으로 resume하지 않는다.
   v121 실제정적25/25·이동6/7, Woman3 running1ground deficit이다. final_validation은1누락 FAIL이며
   CPU31/54GT·최종runtime resume481raw불변/추가capture0·334tests PASS와 구분한다. 모든moving 성공 보장은 아니다.

CURRENT/EVIDENCE/PLAN/HISTORICAL 상태를 구분한다. 현재 학습 전달본은
`base3_online_highway_motion_v2`이며 nearest-state V1을 자동 선택하지 않는다.
