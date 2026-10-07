# Base5 독립 5K — CURRENT

2026-10-07 / 정적 미션 확장 v111. Base5 생성·수집·변환의 현재 진입점이다.
**Sunny13시 직접5K**, Base3 보강이 아닌 Base5-only single-frame OD 학습용이다.
5K 시나리오 생성·전수 검사와 대표 live/resume/CPU 검증은 완료했다.
사용자가 현재 MR 기본값과 수집 환경을 Sunny 13시로 확인했다.
아래 미시작/승인 대기 기록은 v108 사전 보고 시점이며 현재 수집 진행률을 뜻하지 않는다.
이번 환경 정정에서는 기존 수집을 중단·재시작하거나 본학습을 수행하지 않았다.

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

새 manifest: `~/morai_cap/base5_v2_261007/final_5k_static_all_v111_ready/manifest.json`.
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
