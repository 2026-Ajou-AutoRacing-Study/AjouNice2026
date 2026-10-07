# Base5 독립 5K — CURRENT

2026-10-07 / 정적 single-capture QA v115. Base5 생성·수집·변환의 현재 진입점이다.
**Sunny13시 직접5K**, Base3 보강이 아닌 Base5-only single-frame OD 학습용이다.
v113의 5K 생성·전수 검사와 대표 live/resume/CPU 검증 이력은 보존한다.
현재 v115 코드로 본수집하려면 새 5K manifest/output이 필요하다. 구 v113 명령을 그대로 실행하지 않는다.
사용자가 현재 MR 기본값과 수집 환경을 Sunny 13시로 확인했다.
아래 미시작/승인 대기 기록은 v108 사전 보고 시점이며 현재 수집 진행률을 뜻하지 않는다.
이번 환경 정정에서는 기존 수집을 중단·재시작하거나 본학습을 수행하지 않았다.

## 2026-10-08 수동 waypoint 높이 보간 — v116 live 검증 진행

사용자 보정본10개/49waypoint를 원본·EgoNetwork와 함께 hash 검증/백업하고 MR에서 해당 파일만 정리했다.
`2_MN.json`은 없어 missing으로 기록했다. 실제 수동 XY를 횡단보도 축에 투영해 자산별 경로 높이를 보간한다.
world actor Z와 내부 waypoint Y를 구분한다. 길이 밖·실제 경로에서1m 이상은 외삽하지 않는다.
단일 선의 지면 mesh 인증이나 다른 구역 공통 ground로 해석하지 않는다.

5자산×2횡단보도×양방향20회 PASS, 추가 Woman2 running4회 PASS다.
20회 원본-capture wall166.180초/평균8.309초, 실제 Object Z와 경로 보간 높이 최대차2.694mm,
측정 가능한 annotation 하단/지면 최대절대 gap8.583cm다. 원본 point·GT는 보존했다.
추가 empty-terrain/static-reference/height recapture0이며 raw 자체 ground 검사만 사용한다.
근거 부족/작은 오차는 warning, 신뢰 local-plane ±40cm 초과·Z 불연속·domain 밖은 review다.
Ego road support/실제 정지·CollisionData·실제 움직임/GT/가시성 검사는 유지한다.

자산 profile의 pending lookup은 calibration_only에서만 허용한다. 실제 양방향 근거와 source hash가
있는 profile만 LIVE_VALIDATED로 따로 발행한다. 나머지 일반 사람23종의 경로 공유는 개별 live 검증 중이다.
신규 generator opt-in `dynamic_region=intersection-only-v116`은 범용 이동만 standing으로 바꾸며
자산·family/거리 quota는 유지한다. lookup 미포함/범위 밖 경로는 기존 엄격한 준비로 남는다.
기존 default config·구5K manifest/output은 덮어쓰지 않았고 새전체5K/본수집/학습은 미시작이다.
최종 integration/추가 평행·근접 검증은 진행 중이다. 정적4171/이동829는 이전 manifest 수량이다.

## 2026-10-08 이동 캡처 사거리 제한·수동 높이 보정 — 설계 이력

사용자 방향: 이동 외관이 필요한 사람/동물/복합체만 우선 신호등 사거리 두 횡단보도에서 이동 캡처한다.
범용/정적 회피에서는 정지 배치하며, NPC 및 검증된 rigid 외관도 정지 캡처 우선이다.
GPS 음영 저격은 조명/배경을 고려해 후속 별도 설계한다. 공식 출제 구역 제한으로 해석하지 않는다.
현재 런타임/전체5K manifest 변경은 아직 없고 v115 동적 엄격 검사를 유지한다.
Woman2/Dog1/Dog2/Deer1/Ped_Pet1×두 횡단보도=10개 수동 보정용 constant-Z 경로를
MR에 `0_<번호>_<자산>_<횡단보도>.json`+EgoNetwork sidecar로 설치했다. 실제 시나리오 로드는 하지 않았다.
사용자가 UI로 출발/전체 waypoint 접지 후 `1.json`…`10.json`으로 저장하면 원본과 함께 backup하고
자산×경로별 bounded 높이 보간표를 구축·live 검증한다. 아직 lookup 합격/시간 단축은 주장하지 않는다.
다른 pivot에 무조건 공유하거나 경로 밖으로 외삽하지 않는다. 과거4171/829 수량·ETA는 이전 manifest 이력이다.
[수동 작업·정책·후속 검증](261008_base5_intersection_manual_ground_plan.md)이 다음 단계다.

## 정적 지면 QA 경량화와 실제 비용 — v115 우선

사용자 승인에 따라 정적 장면은 이미 취득한 LiDAR의 local ground 검사만 사용한다.
`static-single-capture-advisory-v115`가 명시된 장면에만 적용하며, 모든 actor가 static/standing이어야 한다.
plane 미확인·중간 정도 annotation 높이 오차는 warning으로 보존한다. 접지 인증으로 표시하지 않는다.
신뢰할 수 있는 local plane 대비 annotation 하단이 +0.40m 초과/−0.40m 미만이면 review다.
정적 지면 확인만을 위한 Ego 근접 이동·empty terrain·높이 재촬영은 하지 않는다.
심각한 높이 실패는 부족분/기존 bounded same-quota replacement로 처리한다.
가시성·GT 누락·Ego 최종 road footprint/정지·CollisionData·raw/provenance 계약은 유지한다.
하나라도 이동 actor가 있거나 calibration_only이면 기존 엄격한 독립 ground/Z 경로를 유지한다.
원본 센서/GT는 변경하지 않고 core ground 결과와 warning/최종 review를 따로 저장한다.

고정seed261007115의 family-stratified 신규40장면: 39승인/1부족분,
실제43촬영(재배치3회 포함), parent wall271.322초, 승인당6.957초.
부족분은 고주로 후방6–10m 혼합의 `population_visible_actor_deficit`이며 실패를 성공으로 세지 않았다.
정적 ground 추가촬영0, 이동/terrain calibration 촬영0, 안전 미확인 session0이다.
plane 미확인36건은 actor×capture warning 수이며 36장 또는 부유36건이라는 뜻이 아니다.
기존 실제 Boy1 부유+57.812cm 근거는 새 정책에서도 severe review로 남았다.
완료resume: sensor935파일의 hash/inode/mtime·선택ledger·누적wall 불변, 추가촬영0.
CPU 변환39장(train32/val7), Vehicle37/Pedestrian9/Obstacle22/Animal4=72GT; raw 보존.
Base5 단위검사272개PASS다. production 모델/추론/threshold·planning/control/localization은 미변경이다.

정적4171장 조건부 외삽은 family 가중8.115h(단순8.060h)다.
이전 v107 정적10장 표본의13.4h보다 짧지만 표본/생성기/QA 계약이 다르므로 순수 인과 A/B 개선율은 아니다.
운영 예상은 여유 포함 약8–10h로 보며 실제 bulk 누적rate로 갱신한다. 시간 보장이 아니다.
이 값은 이동829장을 포함한 전체5K 시간이 아니다. 동적 준비비용은 이번에 새로 측정하지 않았다.
근거: `~/morai_cap/base5_v2_261007/static_light_v115/`의 `static_cost.json`,
`pilot40_offline_audit.json`, `complete_resume_verification.json`, `collection40/`.
[세부 근거](261007_base5_static_height_evidence.md#정적-qa-경량화와-40장면-실측--v115).
다음은 새 v115 전체5K manifest/전수 audit 준비 및 사용자 본수집 승인이다. 본수집/학습은 미시작이다.

## Ego 도로 지지와 실제 정지 — v114 유지

기존 `b5v2_01502`(manifest index1258)의 실패를 재현했다. 큰 회피 객체의 가상 우회 시점에서
Ego가 도로 anchor 기준 옆으로 약7.8m 이동했지만 원래 anchor의 Z/link를 계속 사용했다.
실제 최종 XY는 가까운 물리 MGeo road centerline에서 약4.45m 떨어진 roadside estimate였다.
객체를 제거하거나 초기 기어만 Park로 바꿔도 수평 이동17–29cm·Z 변동45–62cm가 남았다.
같은 yaw에서 Ego를 도로 기준점으로 옮기면 객체 유무 모두 안정됐다.
도로 밖 배치에 따른 물리적 지지 불안정이 확인됐으며, 상세 지형/서스펜션 기전까지 확정하지는 않았다.

- 생성기: 최종 Ego rear-axle XY와 회전된 footprint의 도로 지지를 검사한다.
  `roadside_estimate`는 passive actor 높이 후보로는 남지만 Ego의 도로 지지로 인정하지 않는다.
- 정적 미션의 가상 우회 Ego가 벗어나면 Ego와 primary를 같은 XY만큼 함께 옮겨
  상대 위치·yaw·BBox 간격을 유지한다. 지지되지 않는 yaw는 같은 자산/phase/quota로 bounded retry한다.
  최종 Ego XY에서 높이를 다시 구하고 actor 높이도 기존 최종 XY 정책으로 독립 초기화한다.
  실제 controller가 주행한 회피 궤적이나 객체 접지 인증으로 해석하지 않는다.
- 수집기: 실제 scenario의 Ego XY/yaw를 로드 전에 재검사한다.
  Park ACK 이후0.3초 실제 정지 witness를 확인한다(XY/Z span와 XY/수직 속도 각각<0.02).
  최대4초만 기다리고 캡처 직전에 재확인한다. 드리프트8cm·CollisionData 계약은 완화하지 않는다.
- 실패 좌표·속도·seq/source stamp·서비스 응답·원본 hash를 저장한다. 통신/정지 불확실은 halt다.
  CollisionData 접촉은 기존 physical-collision 복구 분류를 유지한다.
- 세션 계약v7은 도로/경계 지도와 관련 runtime hash도 고정한다. 기존 raw/manifest/output은 보존한다.
  새 코드로 구 output에 resume하지 않는다. v113 재현 revision은 `10578aa8`이다.

기존 5K의 설계 셀을 파일 저장/본수집 없이 전수 재구성한 결과5000/5000 생성 가능,
기존family와122종 primary 최소4회를 유지했다. 회피 쌍390개를 도로 지지 위치로 옮겼다.
새 5K 동결 manifest를 생성한 결과나 전체 지형/가시성 live 인증은 아니다.
실제 A/B·수정본 캡처·한계는
[Ego 드리프트 근거](261007_base5_static_height_evidence.md#ego-드리프트-원인-분리와-보완--v114)를 따른다.
다음 실행 기준은 위 v115다. 본수집·학습은 이번 작업에서 시작하지 않았다.

## 최종 객체 XY 기준 초기 높이 — v113 이력

일부 좌우/혼합 배치에서 XY를 옮기고도 이전 anchor Z를 사용하던 경로를 보완했다.
정적/standing actor는 최종 XY에서 실제 MGeo 도로 선분 XYZ를 보간하고,
그 link에 연결된 차선 경계가 있을 때만 횡경사를 보간한 뒤 자산 pivot 높이를 더한다.
경로용 `lazy_init`/lane-change 가상 link 208개는 지면 근거에서 제외한다.
전역 삼각분할·일괄 +Z 여유·raw cloud/annotation 보정은 하지 않는다.
지도는 지형 mesh/collider가 아니며 roadside/unsupported의 실제 접지는 계속 검사한다.
F2 저장 pivot의 동일 계산 방식 초기 후보는 81/122종이며 접지 인증 수가 아니다.

새 동결 manifest:
`~/morai_cap/base5_v2_261007/final_5k_final_xy_ground_v113_final/manifest.json`.
SHA256 `86f2d7e7c8dd2c319623ecb8343ac203caa951c60049727f98b1594ab92bd758`.
5000 unique / OFFLINE_PASS / 오류0, 122종 planned-primary 최소4, train4382/val618,
정적4171/이동829를 유지했다. v112의 Ego/객체 XY·yaw·spec·split·이동 경로는 모두 보존했다.
이동 actor 1271개는 변경하지 않았고 정적 actor 9371개 중7671개의 입력 Z가 바뀌었다.
절대 변화 p50 약0.002cm/p95 7.763cm/최대1.207m다. 최대치는 roadside 추정이며 검증된 높이가 아니다.
지도 confidence는 road strip2067/near centerline5187/roadside1138/unsupported979다.
이 confidence를 센서 ground 검사 성공으로 사용하지 않는다.

실제 동일 위치 Dog2 A/B: annotation 하단–지면 +31.274cm → −10.089cm,
old review → new accepted, 입력 Z/실제 캡처를 바꿨으며 GT를 사후 이동시키지 않았다.
정상 근접 Ped_Pet1/ElectricScooter2 old/new 모두 통과했다.
혼합4actor는 원본1회 + Girl1 근접 witness1회 + 원본 raw 재검사로 통과(13.567초), 재촬영0이다.
새 frozen 목록 대표4셀은3승인/1부족분(66.306초); 부족분은 근접 혼합의
`static_ground_reference_pose_or_view_mismatch`이며 primary 자체 ground는 통과했다.
별도 최초 Mixer scene은 캡처 전 `Stationary Ego drift exceeded`로 halt/Park됐고,
최대높이변화 animated 혼합은 지형 witness 확보 불가로 부족분이다. 실패를 숨기지 않는다.
CPU 승인분: 혼합1장 GT4(Obstacle3/Pedestrian1), frozen 차량3장 GT4, raw 보존.
261 Base5 tests PASS. 완료 대표 campaign resume 추가촬영0/3승인1부족분 유지.

정적 QA 비용 제한은 child quality-loop마다 measured-height 재촬영1회·배치 대체1회,
지면 witness 최대8회이며 동일 UID+동일 scenario payload에는1회만 허용한다.
8회는 actor 최대8개에 대한 상한이지 매 장면8회를 찍는다는 뜻이 아니다.
실측 above-ground일 때만 높이 재촬영하며 증거 부족만으로 Z를 낮추지 않는다.
한도 초과는 deficit로 보존한다. 바깥 quota campaign은 기존 최대2round이므로
전체 비용 한도가 한 child 한도와 같다고 해석하지 않는다.
서로 다른 output에서도 같은 ROS master를 동시에 제어할 수 없도록 live lock을 추가했다.
원래 output에는 새 runtime으로 resume하지 않는다. v112 재현은 commit `c67f62c3`이다.
본수집/학습은 시작하지 않았다. 5K 전체 live 인증·고정 ETA는 주장하지 않는다.
세부 실험/실패 원인은 [초기 높이 근거](261007_base5_static_height_evidence.md#최종-xy-기준-보완--v113)를 따른다.

## 동적 장애물 미션 차량 소량 보강 — v112 이력

기존250개 동적 미션 슬롯 중 일반 사람12개를 차량12개로 교체한다.
미션의4.8%,전체5K의0.24%이며 동물/반려동물 동반/유모차/탑승자/dummy 희귀 quota는 유지한다.
차량 단계는 접근6/횡단3/양보2/통과후1이다. 길이6m 이하 일반 차량 pool에서12종을 고르게 선택했다.
Porter2 등 소형 상용차도 포함하며 MiddleSize 출제 한정으로 해석하지 않는다.
대형 차량은 다른 패밀리/정적 미션에서 유지한다. 여기서43종 모두를 찍었다는 뜻은 아니다.

차량은 Single-frame 외관 학습이므로 정지 캡처한다. 실제 차량 이동/trigger/속도 GT 인증은 아니다.
CW000000/CW000002 주변12m 안의 횡단 방향 MGeo 도로 link XYZ/yaw를 사용하며,
횡단보도 보행 축 위에 임의 차량 yaw를 넣지 않는다. road witness를 전수 audit한다.
차량 양보 관측만8–20m로 조정했다. 기존 사람용5–10m와 일부 큰 차종은
거리·비충돌 조건을 동시에 충족하지 못했다. 보행자 등 기존 거리 정책은 유지한다.
metadata/manifest에 같은 거리 정책을 기록하고 같은 primary 자산으로 bounded retry한다.

새 manifest:`~/morai_cap/base5_v2_261007/final_5k_dynamic_vehicle_v112_ready/manifest.json`.
기존v111의4988개 scene payload/hash/metadata는 그대로 재사용하고12개만 재생성한다.
정적 우선 순서는4171→실제이동829이며 기존4160/840 limit를 재사용하지 않는다.
family/미션 총량5000/250, GT/가시성/접지/CollisionData 계약은 유지한다.
기존 manifest/output을 덮어쓰지 않았으며 구output에는 새runtime으로 resume하지 않는다.
v111 재개는 원래 commit `b0997088`, v105/v109는 원래 `08bf6816`을 사용한다.

검증:5000 unique/OFFLINE_PASS/오류0,train4382/val618/test0,
122종 planned-primary 최소4장 유지. 새4개 단위검사를 포함한 결과와 실제 최종 smoke는 아래 후속 기록에 갱신한다.
초기 대표 live `dynamic_vehicle_v112_smoke4`는 접근/횡단/양보/통과후4/4,
32.924초였다. ground/contact gate·종료Park을 유지했다. CPU4frame/Vehicle5GT,제외0.
이 초기 manifest의 entry 양보 거리에 구5–10m가 남아있어 spec의8–20m와 달랐다.
scenario/실제 검사 정책은8–20m였고 entry도 일치하도록 수정해 최종 manifest를 새로 만들었다.
전체 자산/새 지형 live 인증이나 모델 성능 결과는 아니다. 본수집·학습은 시작하지 않았다.

최종 `dynamic_vehicle_v112_final_smoke4`도4/4,32.983초였다. 양보entry/spec8–20m 일치,
4장 실제CollisionData no-contact·ground gate 및 종료Park을 확인했다.
CPU4frame/Vehicle5GT·제외0·raw보존이다. 신규12종을 모두 live 인증한 것은 아니다.
단위검사250개 PASS. 완료smoke의동일명령resume도추가촬영없이완료됐다.
manifest SHA256:`6b675018cad62652c385facb5583682e91a4512fb860e3720659217535be821c`.
후속 정적 자산 상향/하향 배치 반복시험은 별도 계획이다. 자동 낙하·침투 복구를
모든 자산에 가정하지 않으며 현재 접지 gate와 입력 Z 보완 정책은 유지한다.

## 정적 장애물 미션의 전체 자산 대응 — v111

정적 장애물 구간은 Obstacle 클래스만 나오는 구간이 아니다. 공식 출제 범위에 맞춰
일반 차량 43종을 정지시킨 경우, 정지 사람·동물·유모차·탑승자·반려동물 동반 사람도 포함한다.
정적 미션에서는 보조 actor까지 속도 0, waypoint 이동 없음이다. 원본 Instance의
Vehicle/Pedestrian/Obstacle/Animal 및 복합 component 분류를 유지하며 모두 Obstacle로 바꾸지 않는다.

| 정적 미션 주 객체 그룹 | 장면 |
|---|---:|
| 실체 장애물 19종 |224|
| 일반 차량 43종의 정지 배치 |180|
| 정적 차량 에셋 16종 |66|
| 일반 사람 25종 |90|
| dummy 3종 |12|
| 동물 3종 |18|
| 반려동물 동반 사람 1종 |12|
| 유모차 3종 |18|
| 탑승자·이륜차 복합 9종 |30|
| 합계 |650|

기존 obstacle 미션 600과 other_vehicle 미션 50의 슬롯 안에서 변경한다.
위 그룹은 GT 클래스별 총량이 아니라 **주 객체 배치 quota**다. family는 장면 맥락이다.
전체 5K 중 650장면만 새로 생성하고 기존 4350 scene row/payload는 그대로 재사용했다.
회피 approach/entry/alongside/clearing, 좌우·경계 간격·1–3 actor 정책을 유지한다.
122종 모두 정적 미션 primary 최소 3장, dataset 전체 planned-primary 최소 4장이다.
새 split은 train4381/val619/test0이며 실제 가시성·접지는 별도 capture gate를 통과해야 한다.

v111 보존 manifest: `~/morai_cap/base5_v2_261007/final_5k_static_all_v111_ready/manifest.json`.
전수 OFFLINE_PASS/5000 unique, planned coverage PASS다. 대표 live 결과는 아래 후속 기록을 따른다.
기존 v105/v109 output·캡처·contract는 보존한다. 생성기 코드 변경으로 **새 runtime을
기존 output에 resume하지 않는다**. 기존 수집을 재개할 때는 원래 revision을 사용한다.
본 수집은 자동 시작하지 않았다. 이전 비용 표본의 13.4h/45.7h는 새 분포의 확정 ETA가 아니다.

### v111 대표 검증 결과

- `static_mission_all_v111_smoke9`:9장면 중7승인/2부족,145.136초.
  Scooter_01/K5/정적Grandeur/Woman5/NCAP_EPTa/Dog2/BabyStroller2가 통과했다.
  Ped_Pet1·ElectricScooter2의 접근 장면은 bounded 재배치 후에도 지면 관측 근거가 부족했다.
  부유가 확정된 것이 아니며, 해당 장면을 성공으로 세지 않았다.
- `static_mission_composite_close_v111`:같은 두 자산의 별도 alongside 장면2/2,
  11.291초. 원래 접근 장면을 복구한 것으로 합산하지 않는다.
- 승인9장 모두 정지 설정·ground gate·실제 CollisionData no-contact 근거를 유지했다.
  122종 전체 live 인증은 아니며, 모든 새로운 접근/근접 지형이 해결됐다는 뜻도 아니다.
- 첫7승인 CPU 변환:7frame/8GT(Vehicle2/Pedestrian3/Obstacle2/Animal1).
  가시성 근거가 부족한 Pedestrian component1개 제외,raw 보존. 정적 미션이라고 GT를 Obstacle로 덮어쓰지 않았다.
- 새manifest SHA256:`499ba5663ef824d3ebbdb2a5c25a132fa303a12baacbe8c361ee833f5599ab56`.
- 단위검사246개 PASS. 완료 smoke9 resume에서 selected ledger SHA와 누적145.136초가
  불변이며 추가 촬영0이었다. 전체 audit 및 신규/기존 output 구분을 확인했다.
  Production detector/tracker/학습은 미변경이다.

## 읽을 문서

1. 이 CURRENT: 현재 계약·실행 경계.
2. [5K 구성/진행표](261007_base5_direct_5k_current_plan.md).
3. [현재 운영 런북](261007_base5_5k_capture_runbook.md): rc부터 resume/대시보드/변환까지.
4. [v105 근거](261007_base5_final_gates_v105_evidence.md): 실제 실패·보완·결과·비용·한계.
5. [Sunny13시 정정](261007_base5_sunny13_environment_correction.md): 실제 환경과
   잘못된 11시 선언 구분, 기존 resume 보존 및 새 header 경로.

[v100 이전 누적 이력](BASE5_V2_HISTORY_THROUGH_V100.md)은 과거 분석이 필요할 때만 읽는다.
v45/10K 명령·18.7h 등 과거 ETA를 현재 명령/예상으로 사용하지 않는다.
구 halted 수집32장·과거10K manifest·실패/pilot/raw는 보존했다.

## 구성

| 패밀리 | 범용 | 미션 | 합계 |
|---|---:|---:|---:|
| 고주로 차량 |900|600|1,500|
| 로터리 차량 |100|250|350|
| 기타·정적 차량 |100|50|150|
| 실체 장애물 |900|600|1,500|
| 사람·동물·복합 |1,000|250|1,250|
| 빈 배경 |250|0|250|
| 합계 |3,250|1,750|5,000|

범용positive3K는 single900/regular1,500/dense600(30/50/20%). 크기 비용2/3/6/8,
regular10–14점/dense16–22점/최대8actor. 배경250장은 별도다.
미션은 상황별1–3개이며 범용 점수로 억지 증원하지 않는다.
복합 actor의 component 수와 배치/visible-positive parent 수를 구분한다.
범용혼합은 최소2개·배치 parent 절반 이상 positive여야 승인한다.

122종 pool 유지: vehicle43/static vehicle16/obstacle19/person25/dummy3/animal3/
pet-person1/stroller3/rider9. 열린 G80은 공식 출제 제외, 문 닫힌 G80은 유지한다.
WoodBox/CargoBox 및 MR 복구 자산을 포함한다. 배경 나무/시설은 신규 positive quota가 아니다.
종류당 primary 최소4장 계획을 확인했으며 실제 accepted-visible primary4장도 별도로 감사한다.
이 하한은 학습 충분성 보장이 아니다.

## 배치·동작

- Ego는 장면마다 도로 anchor+기존 yaw/횡 offset, 정지·Park·1capture다.
  NPC는 실제 경로 position/yaw이며 임의 횡/yaw jitter를 주지 않는다.
- 고주로 범용은 샘플 despawn 이후를 포함한 허용 도로를 유지한다.
  미션 NPC만 샘플 spawn/despawn corridor로 제한한다.
- 범용 근접175셀, 차량 미션 근접133셀(고주로66/로터리67).
  BBox 경계 간격과 actual annotation 간격을 구분한다. collider 인증은 CollisionData로 별도 검사한다.
- 정적 회피는 approach/entry/alongside/clearing·좌우 관측이며 alongside240obstacle/
  20static-vehicle 셀이다. unknown geometry는 보수적 간격을 사용한다.
- 동적 미션250셀은 MGeo CW000000/CW000002를 기준으로 접근113/횡단62/양보50/통과후25다.
  과거 starter 기준 Ego −35~+55m 범위를 미션 분포로 사용하지 않는다.
  모든 대회 진입 경로를 구현·인증했다는 뜻은 아니다.
- 사람·동물 등 관절 외관 자산은 natural-target 실제 이동, rigid/fixed-pose 자산은 정지다.
  범용 동시 이동은 최대2actor. **주 객체 walk/run은 유지**하고 이후 보조 actor만 standing으로
  제한한다. 미션에는 이 제한을 적용하지 않는다. 속도 회귀/temporal 학습을 위한 정책이 아니다.
- 동적 경로는 empty LiDAR 지면+같은 target의 static annotation으로 높이를 준비한다.
  정적 보조 객체도 동일 독립 지면 참조에 포함한다. 전체 footprint support,10cm height-step,
  실제 이동/최종 Z/contact를 확인하고 unsupported는 성공으로 넣지 않는다.

## GT·안전·실패

- raw LiDAR Instance geometric center/size/rotation/component를 보존한다.
  Object_topic pivot/catalog 보정을 sensor GT에 다시 적용하지 않는다.
- `lidar1_camera0`: LiDAR≥1점 또는 실제 camera Instance visibility 근거.
  0점 camera-visible을 일괄 제거하지 않고, 안 보이는 객체를 무조건 positive로 넣지 않는다.
  현재320×896 bottom crop에서Instance≥16px·최소extent4px다. 입력crop을바꾸면
  raw Instance에서가시성을재감사하며그변경만으로재수집할필요는없다.
- ROI는 LiDAR/Ego[-100,-30,-3,100,30,2]. 도로 접선은 배치용이며 GT/모델 ROI 기준이 아니다.
- Ego pose·신선한 ROS·Park·CollisionData·파일완결·접지·이동/Z·가시성 gate는 유지한다.
  근접 blind spot/낮고 작은 물체를 학습용 가짜 GT로 메우지 않는다.
- fresh primary가 확인된 정적 secondary 누락은 진단 review→동일quota 재배치다.
  missing primary/mover·통신/Park 미확인은 halt. 누락 자체는 성공이 아니다.
- 확인된 접촉은 실패/부족분. 안전 Park·새 scene 상태가 확인된 경우만 다음으로 진행한다.
  unresolved quality는 bounded retry 후 부족분, calibration/review/failed는 학습에서 제외한다.
- safe paused는 같은 코드/manifest hash로 resume. halted는 강제 resume 금지다.
  완료 resume는 증거 검증 전용·추가 촬영0이며 부족분은 별도 보충 campaign이다.

## 최종 준비 산출물 / 환경 정정과 기존 수집 보존

- 원본 `~/morai_cap/base5_v2_261007/final_5k_v105/manifest.json`.
- 기존resume 원본 `~/morai_cap/base5_v2_261007/final_5k_ordered_v105/manifest.json`.
  원본 SHA256: `407cc518863d496b770a87d9cd75c3ecd7989673449c1e7f854bd3fc90b8bb81`.
- 신규 수집용13시 정정본:
  `~/morai_cap/base5_v2_261007/final_5k_ordered_sunny13_v109/manifest.json`.
  SHA256:`1b8a6ced7d2af06631e1cbd2897a4464adc897ef833023f6f4f373e5c0d04523`.
- v109 당시 config:`base5_independent_5k_sunny13.json`의 아래 SHA는 당시 버전이다.
  현재 v111은 정적 미션 policy가 추가돼 이 SHA와 다르다.
  SHA256:`d58e15791a32005655da1e343e109a718646b0859bcd4fdb2918d85e03d35948`.
  기존 generator 기본 경로의 `sunny11` 파일은 동일한 13시 내용의 호환 alias다.
- 5000 scene row/geometry/metadata/split은 정정 전후 불변이다.
  이미 수집 중인 명령은 원본 manifest/output 그대로 resume하고 정정 기록을 함께 유지한다.
  정정본은 새 output에서만 사용하며 중간에 갈아끼우지 않는다.
- 5,000 unique payload, train4,387/val613/test0, OFFLINE_PASS/오류0.
- 순서만 정적4,160→실제 이동840으로 변경, 모든5000payload/quota/split 보존.
- 122종 planned coverage PASS/min4. 전체 sensor effective coverage는 아직 NOT_CAPTURED다.
- 최종 통합 `final_pilot_v105`는 별도 검증이다. 최신 수치·resume·CPU·시간은
  [근거 문서](261007_base5_final_gates_v105_evidence.md)의 동결 결과를 따른다.

## 결론·잔여 경계

- 차량 근접7/7, 최종 통합18셀 중15승인(정적12/12·실제이동3/6), CPU57GT다.
  후속 독립 지면 보완은 이전 실패3셀 중1셀을 추가 복구했다. 전체 성공률 추정이 아니다.
- 고정seed 비용 표본은12중11승인(정적10/10·실제이동1/2),254.250초다.
  정적11.60초/승인,동적138.22초/승인으로 준비·실패·QA 비용을 포함한다.
  계획4160/840으로 단순 외삽하면정적부분13.4h/전체45.7h·원본과QA약75GiB다.
  동적표본2개뿐이고부족분보충·지형차이에민감하므로약속된ETA가아니다.
- **배치·GT·안전·resume 경로 준비와 모든 동적 장면의 접지 해결은 다르다.**
  일부지형은독립support/참조불일치로bounded retry 후부족분이다.
  정적먼저4160→이동840순서는준비했지만,전체5K를빠른야간수집이라고권하지않는다.
  사용자보고후정적우선진행또는동적준비비용추가최적화범위를정한다.
- 신규coverage의`--require-effective-complete`는현재소량15장에대해의도대로거부한다.
  아직전체122종의actual-positive4장씩이충족된dataset은아니다.
- 원본/campaign hash는버전별이다. v108로수정한코드를구v105/v107 output의resume에섞지않는다.
- 최종v108 fresh회귀4/4·4class10GT·완료resume추가촬영0,단위검사238개PASS다.
  이는남은unsupported동적조건을모두해결했다는의미는아니다.

Production E25·threshold0.40/0.15/0.10·tracker·planning/control/localization은 미변경이다.
새 모델 threshold는 학습 후 동일 held-out mission rosbag의 Detection/Tracking/Map-CV로 결정한다.
오프라인 split은 neighboring-view leakage가 남아 최종 일반화 성능의 근거가 아니다.
전체5K 물리 검증/모델 성능/4090 full-stack 실시간성은 이번 소량 수집 검증으로 인증하지 않는다.
