# Base5 독립 10K v2 — CURRENT / 10K 생성·소량 live 검증

2026-10-06. 이 문서를 Base5 생성·캡처·변환의 최신 진입점으로 읽는다.
기존 v1/E25 보강 실험과 구분한다. **시나리오 10,000개 생성은 완료했지만,
소량 live는 아래처럼 통과/검토로 나뉘었다. 본 수집과 본 학습은 시작하지 않았다.**

## 최신 v18 — 동적 거리 수정·live 복구 / 본 수집 미시작

2026-10-06 재연결 후 기존 one-load 정적1장 통과. pause/actor-only load는 재사용하지 않았다.
새 motion 경로의 대표16장 재캡처는 **15후보 /1가시성 검토 /0실행실패**다.
근접 회피8장, 전방60–80m/후방80–99m 차량, 로터리, 빈 배경을 포함한다.
추가 전방 초근접 유모차·Bike3도 통과했고, AI police는 아래 존재 확인 수정 후 재캡처 통과했다.
전체 자산이나10K live가 검증됐다는 뜻은 아니며 본 수집·학습은 미시작이다.

- 일반 사람 요청5–10m: 실제9.526m,33point. 유모차 요청3–5m:
  component3개 실제3.956/3.973/3.989m,98/95/24point. 준비 대기 중8m 이탈 문제를 수정했다.
- 동적 자산은 Ego 기준 거리 원에 접하는0.5–4m 구간을 왕복한다. 동일 endpoint를
  연속으로 연결한64회 유한 왕복이며, respawn/teleport/런처 loop 기능에 의존하지 않는다.
  객체가 움직이는 외관을 확보하는 single-frame 수집 전용이다. 대회 이동 경로, temporal
  학습, velocity label로 사용하지 않는다. 실측 displacement가 없으면 캡처 실패다.
- endpoint·중간점의 거리 cell/ROI와 방향, 회전 중 heading까지 포함하는 보수적 swept
  envelope를 검사한다. 여러 객체의 이동 envelope끼리도 이격을 유지한다. 이 envelope는
  추정 geometry이며 실제 CollisionData·동기 annotation 검사를 대체하지 않는다.
- 전방 초근접 유모차는 실제4.985/5.064/5.077m였다. 기존 annotation 거리 audit의
  ±0.5m tolerance 안에서 통과한 값이며 모두 엄밀히5m 미만이라고 해석하지 않는다.
- Dog2는 같은15–29m/좌측 조건 재배치에서 한 번 통과했지만, 같은 배치 반복에서는
  실제24.565m·0point·camera crop 근거 부족으로 다시 검토였다. 자산 삭제·빈 배경화·GT
  강제 positive 처리를 하지 않는다. 가시성 실패는 raw 보존 후 동일 cell 재배치/보충 대상이다.
  timing 수정이 sparse 관측까지 해결한 것은 아니다.
- police1은 `aiPedstrianList` 전용 AI destination 형식으로 일반 waypoint API와 다르다.
  자산은 포함하되 움직이는 AI 경로는 미검증이므로 정지 외관으로 유지한다. requested motion과
  effective standing은 metadata에 구분되고 manifest는 effective motion을 사용한다.
  실제 annotation10.632m·32point가 있으나 Object_topic에는 해당UID가 없었다.
  정지 AI 자산만 positive sensor annotation으로 존재를 입증할 수 있게 좁은 예외를 추가했다.
  일반 객체 누락·동적 AI·센서 근거 없는 AI는 여전히 검토다. 충돌·지면 gate는 완화하지 않았다.

최신10K 후보: `/home/luv/morai_cap/base5_v2_261006/scenarios_10k_v18d/manifest.json`.
전수검사 OFFLINE_PASS:10,000 unique payload/오류0, train8,611/val1,389.
기존 family/mission/거리/좌우/근접 gap quota 유지. 시나리오는 MR에 일괄 설치하지 않았다.
이전 v18/v18b/v18c 부분 생성 실패본은 진단 근거로 보존하며 본 수집 manifest가 아니다.

주요 증거 경로:

- `.../shuttle_pilot_v18d/manifest.json`, `.../shuttle_capture_v18d/smoke_report.json`
- `.../shuttle_extra_pilot_v18/manifest.json`, `.../shuttle_extra_capture_v18/smoke_report.json`
- `.../ai_presence_capture_v18/smoke_report.json` (새 presence 계약·subset 재실행 no-op 통과)
- 공통 prefix: `/home/luv/morai_cap/base5_v2_261006/`

대표16장 평균5.234초/장·p50 4.959초, raw80,845,627bytes. 단순 외삽10K 약14.5시간이며
보충·실패·환경 부하는 별도다. info 변환은 train12/val3, GT19개 통과했다.
`artifacts/datasets/base5_v18_shuttle_smoke/`이며 학습은 수행하지 않았다.
단위검증30개 통과, dashboard HTTP에서16완료/15후보/1검토를 확인했다.
본 수집 승인은 별도다. 검토 재배치 보충과 provisional GT 정책 확정 없이10K positive를 보장하지 않는다.

presence gate 버전이 session contract에 포함된다. 이전 safety policy 세션과 섞지 말고 새
output을 사용한다. 같은 계약의 resume은 완료 raw/ledger를 변경하지 않는다. 이번 subset
resume report는 실제 선택 indices를 전달하므로 첫 N장으로 잘못 재개하지 않는다.
일부 소량 세션 종료에서 Park 응답 timeout 경고가 있었다. 초기속도0·제동은 유지했지만,
장시간 unattended 수집 전 종료 gear 확인/복구 신뢰성은 별도로 점검한다.

## v17 — 전체 후보 재생성 통과 / 동적 live 검증 중단 이력

최신 후보는 `/home/luv/morai_cap/base5_v2_261006/scenarios_10k_v17/manifest.json`이다.
v16에서 확정한 concrete-barrier/HD65 배치 geometry를 전체에 반영했다.
`offline_audit.json`: 10,000장/10,000 unique payload, 오류0, train8,618/val1,382.
패밀리·미션·거리·회피 좌우/phase/gap quota를 전수 검사했다. 기존 v15/v16 원본은 보존했다.
**이는 offline 후보 통과다. 10K 실제 캡처 승인 또는 live 통과가 아니다.**

v16 검토3건을 원본으로 좁힌 결과:

- worker_m_1: 캡처 직전 pivot은 초기 배치에서7.797m 이동했다. 요청5–10m,
  actual annotation center16.954m. 준비 대기 중 움직임이 거리 분포를 깨뜨렸다.
- BabyStroller2도 요청3–5m에서 실제7.69–8.31m였다. component별 center를
  유지하며 오차 허용 범위를 늘리거나 GT를 옮겨 승인하지 않는다.
- Dog2: actual22.63m, LiDAR0point, native camera instance6pixel, 모델 crop0pixel.
  객체가 존재하지만 현재 입력에서 positive 근거가 없는 사례다. Dog2 자체를 제외하지 않는다.

대기 중 움직임을 막는 두 방식을 실제 시험했으나 채택하지 않았다.

1. `ScenarioLoad.set_pause=True` 후 Event option8/False 재개: 응답 set_pause는
   계속True였고 실제 움직임 재개도 확인되지 않았다. 후속 시험은 Event timeout으로 중단했다.
2. Ego만 로드·모드 확인 후 객체만 별도 로드(network/Ego reload false): 현재 MR 로그에
   network disconnect가 기록됐고 Ego/Object/Collision freshness 실패로 즉시 중단했다.

실패한 변경은 기본 runner에서 제거했다. 프로덕션 runtime은 변경하지 않았다.
실패가 가시성/거리 정책을 완화해야 한다는 근거는 아니다. 본 수집은 보류한다.
재연결 후 기존 정상 one-load 경로를 먼저 확인하고, 동적 캡처 phase/경로와 가시성
재샘플 전략을 검증해야 한다. pause/선택 로드를 다시 쓰려면 런처 지원부터 독립 확인한다.

증거: `/home/luv/morai_cap/base5_v2_261006/dynamic_timing_report_v17.json`,
`dynamic_pause_probe_v17/`, `dynamic_pause_probe_v17b/`, `dynamic_staged_probe_v17/`.
마지막 폴더의 `Player_snapshot.log`에 런처 로그를 보존했다.
재집계 도구는 `scripts/report_base5_dynamic_timing.py`이며 ROS/모델을 실행하지 않는다.

공식 pause/option8 정의는 [MORAI ROS 프로토콜](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R1.0/ros-1)에 있다.
현재 MR의 실험 결과를 공식 지원 여부 전체에 대한 결론으로 일반화하지 않는다.

## v16 safety/resume smoke — 2026-10-06

새 runner로16장 실제 캡처: **13후보 /3검토 /0실행실패**. 본10K 수집은 미실행이다.
manifest: `/home/luv/morai_cap/base5_v2_261006/safety_pilot_v16/manifest.json`.
output: `/home/luv/morai_cap/base5_v2_261006/safety_capture_v16/`.
정량·resume 증거는 `smoke_report.json`, 원본은 captures/와 incomplete_attempts/에 있다.

- 근접 회피8장 전부 후보. annotation gap은 concrete-barrier10.98cm,
  Grandeur12.72cm, HD65 20.04cm, Barricade02 32.54cm였다. 모두 대상 존재와
  동기 GT/센서 evidence가 있고 관측된 CollisionData 구간에 접촉 상대가 없었다.
  위 gap은 실제 객체 annotation과 Ego4.8×2.0m footprint proxy 사이 거리이며,
  물리 collider 간격을 직접 측정한 값은 아니다.
- concrete-barrier와 OBJ_Hyundai_HD65는 기존4개 정지 캡처로 geometry 참고값을
  보정했다. `base5_stationary_geometry_mr_261006.json`의 크기/offset을 배치 추정에만
  적용한다. 학습 annotation과 point는 변경하지 않는다. reference yaw는 MR heading
  -1.347° 계약과 scenario yaw를 대조하며, 이 보정을 annotation에 다시 적용하지 않는다.
- 전방71.78m 차량4 LiDAR point·후방88.05m 차량5 point도 저장/후보 통과했다.
  최종 positive 정책은 provisional이며 원거리 detector 성능이 검증됐다는 뜻은 아니다.
- 일반 사람은 요청5–10m에서 실제16.95m, 유모차는 요청3–5m에서 실제7.69–8.31m로
  이동하여 검토다. Dog2는 annotation이 있지만 LiDAR0point/camera evidence 부족으로
  검토다. 상대 motion/통신 대기와 캡처 timing 보완이 여전히 필요하다.
- 평균6.079초/장, p50 4.923초, 최종 raw 81,606,035bytes.16장 표본 외삽10K 약16.9시간.
  retry/실패 보충/환경 부하는 별도이며 원래48장 표본과 구성도 다르다.
- 정상2장 후 중단, 세 번째 raw 이동 후 의도적 중단, 재개, 완료 후 재실행까지 검증했다.
  완료16장의 hash/mtime과 ledger는 불변, 중단 raw는 별도 폴더에 보존했다.
- 학습 info 변환 smoke도 통과했다. `artifacts/datasets/base5_v16_safety_smoke/`에
  train12/val1/test0, GT15개(Obstacle4/Vehicle7/Pedestrian2/Animal2)가 저장됐다.
  검토3장은 제외했고 raw는 보존했다. 학습 또는 모델 성능 평가는 수행하지 않았다.
- Base5 관련 단위검증26개 통과. Dashboard HTTP에서 complete16/후보13/검토3을
  확인했다. status는 camera audit 전체를 반복 직렬화하지 않는 compact summary이며
  상세 annotation·collision 증거는 ledger와 captures에 보존한다.
- 수집 gate는 실제 대상 존재·annotation·CollisionData 이력을 사용한다. 측정된 Z 안정성
  (<4cm span)을 stationary support proxy로 기록한다. 포장면/물리 mesh 지지 인증은 아니다.
  annotation0.15m gate는 충분히 관측된 무충돌 이력이 있을 때 hard reject로 사용하지
  않으며 actual gap/overlap은 audit에 보존한다. 다른 actor 간 겹침 검사는 유지한다.

기존 생성10K v15는 보존했다. 이번 v16은 보정된 대표16장 subset이다. 본 수집 전에는
동적 timing/가시성 검토를 보완하고 geometry 업데이트를 전체 후보 deck에 적용·offline
재검사해야 한다. 이전 완료48장/회피 pilot을 새 safety 계약으로 자동 승인하지 않는다.

충돌 gate 근거: [Network 런북의 정지/1km/h 대조](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md).
정지 근접 배치에서는 객체 로드 거부가 재현됐고, 정상 배치 후1km/h 접촉 두 번에서
CollisionData의 상대UID8500이 검출됐다. 정지 후 상대가 사라져 캡처 시점 한 메시지로
충돌 이력을 대체할 수 없다. 새 runner에는 이 이력 수집 gate가 연결됐다.

## 목적과 구현 범위

- Base3와 섞거나 E25에서 시작하는 것을 전제로 하지 않는 독립 single-frame OD corpus다.
- 1 scene = 1 동기 캡처. Ego는 정지시키고 동적 자산은 움직이는 외관을 캡처한다.
- Temporal input, detector velocity target, GT object-motion input은 없다.
- MR만 사용하고 scale=1이다. 열린 G80는 기존 지면 실패 때문에 보류하며 일반 G80,
  WoodBox, MR 복구 자산은 포함한다. Tree/Fire/맨홀/Speed_Bump/일반 표지판은 positive가 아니다.
- Production E25, 0.40/0.15/0.10, tracker, planning/control/localization은 변경하지 않았다.
  새 모델 threshold는 학습 후 dev/held-out 평가로 정한다.

## 생성된 할당량

| Primary family | 범용 | 미션 조건부 | 합계 |
|---|---:|---:|---:|
| 고주로 차량 | 1,800 | 1,200 | 3,000 |
| 로터리 차량 | 200 | 500 | 700 |
| 기타 도로·정적 차량 | 200 | 100 | 300 |
| 실체 장애물 | 1,800 | 1,200 | 3,000 |
| 보행자·동적/복합 | 2,000 | 500 | 2,500 |
| 빈 배경 | 0 | 0 | 500 |
| 합계 | 6,000 | 3,500 | 10,000 |

이는 primary 장면 quota다. 보조 객체가 함께 있어 class별 박스 수와 다르다.
보행자 관련은 일반 사람1,875 / dummy125 / animal125 / 사람+개125 /
유모차125 / 탑승자125다. 풀은 일반 차량43, 정적 OBJ 차량16, 실체 장애물19,
사람25, dummy3, animal3, 사람+개1, 유모차3, 탑승자9를 사용한다.
기타 차량300장은 정적 OBJ150 / 일반 차량150이다.

고주로는 합류600 / 본선1,500 / 병목 이후600 / 출구300이며 primary 전방1,500 /
후방1,500이다. NPC는 실제 MGeo 경로 위치·yaw를 사용한다. 임의 NPC 횡 offset은 없다.
회피 관측 패밀리 외 Ego yaw는 도로 대비 절대0–3° 60%, 3–10° 30%, 10–15° 10%, 부호는 랜덤이다.
Ego 횡 offset은 0/±0.15/±0.30 m이며 좁은 구간은 ±0.15 m와 lane-envelope 검사로 제한한다.

### 고주로 범용 / 미션 NPC 배치 경계

- 범용1,800장: 기존 확장 MGeo 경로를 유지한다. 샘플 NPC 종료 지점으로 자르지 않는다.
  병목 이후·후반 구간도 범용 학습 분포에 포함한다.
- 미션1,200장: primary와 보조 `vehicleList` 객체 모두 샘플의 세 NPC 경로 중
  하나의 startLinkInfo.pointIdx부터 endLinkInfo.pointIdx까지에만 배치한다.
  인접 차선 선택도 이 경계 밖으로 우회하지 않는다. 해당 거리·방향에 가능한 위치가
  없으면 장면을 재시도하며 범용 배치로 fallback하지 않는다.
- 경계는 NPC pivot/reference position 기준이다. Ego까지 같은 종료점으로 자르지는 않는다.
  종료점 이후 Ego가 뒤쪽 NPC를 보는 장면은 가능하다. NPC BBox 가장자리 경계가 아니다.
- 샘플 UID6/8/10의 종료점은 각각 `A2256W001105:83`, `A2256W000422:59`,
  `A2256W000432:246`이다. UID6은 420→179→1105 출구 경로를 사용한다.
  이는 샘플의 **설정상 종료점**이며 실제 despawn timing이나 콘 위치를 새 live 실험으로
  확정했다는 뜻은 아니다. 본선 차종·속도가 샘플과 동일하다고 가정하지 않는다.
- 최신 생성 후보는 `scenarios_10k_v15/`이다. 이전 v12는 고주로 경계 수정 증거로 보존한다.
  NPC 경로/종료점과 sample hash는 metadata/manifest에 남기고 실제 scenario position도 감사한다.

### 거리 — geometric center의 Ego-frame XY 거리

| 대상/방향 | 거리 bin과 비중 |
|---|---|
| 고주로 전/후 | 6–10:10%, 10–30:25%, 30–45:20%, 45–60:15%, 60–80:15%, 80–99:15% |
| 로터리 | 5–10:25%, 10–20:40%, 20–30:25%, 30–40:10% |
| 기타 차량 전/후 | 6–15:35%, 15–30:40%, 30–60:25% |
| 기타 차량 측방 | 3–6:25%, 6–15:50%, 15–29:25% |
| 보행자 관련 전/후 | 3–5:25%, 5–10:20%, 10–15:20%, 15–30:25%, 30–50:10% |
| 보행자 관련 측방 | 3–5:25%, 5–10:20%, 10–15:20%, 15–29:35% |
| 장애물 전방 | 3–10:40%, 10–30:35%, 30–60:20%, 60–99:5% |
| 장애물 측방 | 2–5:50%, 5–15:35%, 15–30:15% |
| 장애물 후방 | 3–10:60%, 10–30:35%, 30–60:5% |

위 거리 deck과 장애물 전방40% / 좌20% / 우20% / 후20%는 **범용 배치 규칙**이다.
회피 미션1,300장은 아래 phase/표면 간격 규칙으로 대체한다. 기존 전역 방향·거리 quota가
그대로 유지됐다고 주장하지 않는다. 범용은 추정 clearance0.3 m 이상을 유지하고
회피 나란히 통과는0.10–0.50 m 후보를 별도로 생성한다. 중심 거리와 표면 간격을 혼동하지 않는다.
현재 ±30 m lateral ROI에서 측방30–50/60 m는 불가능하므로 위처럼 수정했다.
고주로 근접 대형차는 비충돌 가능한 실제 인접 차선/위치를 찾으며 불가능한 cell은
다른 방향으로 몰래 바꾸지 않는다. 512회 재시도 후 실패하면 생성 자체를 중단한다.

### 정적 차량·장애물 회피 관측 — v15

대상은 미션 `obstacle`1,200장 + 미션 `other_vehicle`100장이다. 고주로 주행 NPC와
범용은 이 가상 회피 경로를 적용하지 않는다. 1 scene/1 stationary capture 계약을 유지한다.

| 관측 단계 | 장애물 | 정적 OBJ 차량 | 배치 |
|---|---:|---:|---|
| 접근 | 240 | 20 | 객체보다8–60m+두 박스 반길이 합 만큼 앞선 관측 위치 |
| 회피 진입 | 240 | 20 | 객체 앞에서 smoothstep 횡 이동 구간의 접선 yaw |
| 나란히 초근접 통과 | 480 | 40 | 객체가 Ego rear-axle frame의 좌·우에 있도록 배치, oriented footprint 간격 역산 |
| 통과 직후 | 240 | 20 | 객체를 지나 원래 방향으로 복귀하는 접선 yaw |

각 단계 안에서 **Ego 좌회피/우회피50:50**다. Ego가 좌로 피하면 객체는 오른쪽에 있으므로
`avoidance_side`와 `actual_ego_role`은 반대다. 차량 reference는 후륜축이고 Ego BBox 중심은
1.35m 앞에 있다는 기존 footprint proxy를 유지한다. 긴 트럭에서도 실제 측방 quota를 감사한다.
나란히520장 gap bin은0.10–0.20m25% /0.20–0.30m35% /0.30–0.50m40%다.
JSON mm/0.001° 반올림을 고려해 bin 끝에서3mm 여유를 둔다. 각 장면은 임의 center 거리 대신
원하는 **회전 BBox 표면 간격**을32회 이분 탐색해 맞춘다. 나머지 단계/보조 객체는0.3m 검사 유지.

객체를 먼저 기준 도로에 놓고 Ego를 그 주변의 회피 관측 위치에 놓는다. 장애물 yaw는
도로 기준0/±90°+±8° 후보, 정적 OBJ 차량은 도로 방향이다. Ego yaw/횡 이동은 별도 회피
관측 계산을 따르므로 기존±0.30m lane-envelope를 적용했다고 해석하지 않는다.
이 계산은 연속 제어기의 실행 가능한 경로가 아니라 **독립 관측 자세**다. 곡선·좁은 도로에서
Ego의 포장면/지면 지지가 보장되지 않으므로 다음 live gate가 필수다.

근거는 [기존 실제 회피 bag 분석](../../../docs/bevfusion/260924_STATIC_OBSTACLE_TRACKING_DIAGNOSIS.md)의
Sonata manual02와 Barricade pos04 manual01이다. Barricade의 최저 geometric 간격은0.293m였고,
Sonata는 물리 충돌 없이 annotation OBB의 작은 overlap도 있었다. **이번에 bag 궤적을 그대로
추출·재현한 것은 아니며**, 해당 근거로 설계한 analytical 후보다. BBox overlap 후보는 생성하지 않는다.

`avoidance_observation`에 요청/실제 gap, phase/side/yaw/offset 및 미검증 지면·collider 상태를 남긴다.
이전 v15 동기 캡처 audit는 `avoidance_live_review`의 지면/무충돌 확인과 evidence 경로가 없으면
`avoidance_ground_collider_review_required`로 review 처리한다. 현재 actual annotation0.15m gate도
유지하므로10–15cm 후보는 추가 검토 대상이었다. 최신 v16 gate는 위 절을 따른다.
Captured raw GT를 임의로 줄여 통과시키지 않는다.
통신 연결 성공·파일 저장 성공만으로 이 gate를 승인하지 않는다. 범용의 도로 밖 XY 배치에 대한
실제 지면 검사도 아직 미구현이며 본 수집 전 남은 공통 과제다.

## GT·class 정책: raw 보존, positive 후보 비교

GT는 동기 LiDAR Instance의 center/rotation/size를 직접 사용한다. /Object_topic의
차축→중심 offset이나 -1.347°를 annotation에 다시 적용하지 않는다.
같은 부모 UID의 사람·개·유모차·아기·자전거 GT를 독립 component로 보존한다.

원본 point/annotation을 삭제하지 않고 `lidar5`, `lidar1`, `lidar1_camera0`를 각각 계산한다.
Camera-only 후보는 실제 Instance pixel, model bottom-crop 내 가시성, 최소16 pixel 및
4×4 px extent가 필요하다. 같은 부모·같은 class 다중 component는 카메라 귀속이
애매하므로 zero-point positive로 자동 승인하지 않는다. **최종 GT 정책은 미확정이다.**
역사적12장 fixture에는 zero-point camera-only 사례가 없었지만 새48장에서는3개를
관측했다. 작은 pilot이므로 아직 이 정책의 학습 성능 우월성을 검증한 것은 아니다.

움직이는 동물을 UNKNOWN 강제정지로 묶는 한계 때문에 별도 **4-class 연구 후보**
Vehicle/Pedestrian/Obstacle/Animal을 연결했다. 이전3-class 합의에서 바뀐 실험안이며
production 변경이나 최종 학습 ontology 확정이 아니다. Animal/bicycle의 tracker·planning
runtime mapping 및 기존 class 회귀 검증은 본 학습/승격 전에 별도로 결정해야 한다.
아기는 Pedestrian, 유모차는 Obstacle이며 component 원본 종류도 남긴다.

## 모델 wiring 검증과 production 경계

후보 config: `perception/detection3d/bevfusion/configs/bevfusion_base5_independent_100m.py`.
이미지320×896, positive ROI x±100/y±30 m, batch1, random init, 7D box다.
내부 연산 grid는 stride 정합 때문에 y±32 m로 padding한다. 이것을 positive 범위 확대로
해석하지 않는다. E25를 load하지 않으며 이미지 backbone도 pretrained 없이 smoke했다.

직사각형 BEV에서 기존 정사각형 grid가 숨기던 두 문제를 실제 loss pass로 확인/수정했다.
카메라 BEV XY→YX 전치 및 TransFusion position grid의 flatten/XY 순서를 별도 opt-in
모듈로 고쳤다. SECOND/FPN 정합을 위해 Y600 대신640 voxel을 쓴다.
기존 production 모듈/config는 건드리지 않았다.

5060 Ti 단일 batch의 finite loss와 전체 gradient finite 검사를 통과했다.
Peak allocated3.723 GiB / reserved4.715 GiB였다. 34.684초는 모델 초기화+역전파 smoke
시간이며 학습 throughput/추론 latency/모델 정확도 측정이 아니다. 기존 MR을 끄지 않았다.

## 증거 경로와 판정

로컬 출력 root: `/home/luv/morai_cap/base5_v2_261006/`.

| 증거 | 결과 |
|---|---|
| `scenarios_10k_v15/manifest.json`, `offline_audit.json` | 최신 후보10K offline PASS. 회피1,300장 phase/side/gap quota 및 고주로 sample 경계 검사 통과 |
| `scenarios_10k_v15/v12_payload_comparison.json` | 회피 미션 외8,700장 scenario SHA256 v12와 동일. 범용 및 고주로 미션 유지 |
| `avoidance_pilot_v15/geometry_audit.json` | 8/8 raw 캡처,6/8 primary annotation. actual 표면 gap: Barricade02 0.325m, Grandeur 0.126m. 모든8장 review 유지 |
| `avoidance_gap_probe_capture_v15/geometry_audit.json` | 누락2장의 Ego만1m 외측 이동한 대조군,2/2 primary 복구. 학습용 아님 |
| `avoidance_gap03_capture_v15/geometry_audit.json` | 같은2장의 Ego만0.3m 외측 이동,2/2 primary 복구. actual gap concrete-barrier0.340m, HD65 0.259m |
| 회피 구현 단위 검증 | 8 tests PASS, 회전 barrier/대형 vehicle/좌우 footprint 역산 및 exact phase quota 포함 |
| `scenarios_10k_v12/manifest.json`, `offline_audit.json` | 과거 후보10,000 unique payload, errors0. 미션 고주로1,200장/차량1,568개 sample 경계 검사 통과 |
| `scenarios_10k_v12/v10_payload_comparison.json` | 범용 고주로1,800장 및 다른 장면7,000장, 총8,800개 scenario SHA256 v10과 동일 |
| v12 생성 resample / 단위 검증 | 평균0.6823회, 최대76회; Base5 independent tests7개 통과. 잘린 경로의 원본 pointIdx/initLinkRatio 유지 검사 포함 |
| `scenarios_10k_v10/manifest.json`, `offline_audit.json` | dummy 정지 수정 후10,000 unique payload, quota/거리/추정 footprint 검사 errors0 |
| 동일 manifest split | train8,609 / val1,391, link-group hash |
| 생성 resample | 평균0.594회, 최대76회 |
| `historical_component_replay_v3/` | 기존12장 재파싱,17component, G80 2장 제외하여10장 후보 |
| Repo `artifacts/datasets/base5_v2_pilot/gpu_forward_smoke_v2.json`, `gpu_backward_smoke.json` | loss/gradient finite; 새 v2 수집 정확도 결과가 아님 |
| 동일 Repo 경로 `gpu_predict_smoke.json` | random-init 단일 batch decode14개, finite/4-class label 검사 통과; 정확도 아님 |
| `capture1/`, `capture2/` | 첫 load 뒤 서비스 응답/연결 실패, 새 v2 accepted0 |
| `capture_pilot_retry/`, `pilot_reaudit_v2/summary.json` | v9 대표48장 실제 캡처,41장 후보/7장 검토,원본/이전 audit 보존 |
| `dummy_fix_live/` | v10의 정지 dummy 재캡처1장 통과 |
| `dynamic_timing_fast/` | 거리 실패5장 재캡처,1장 통과/4장 거리 검토 유지 |
| Repo `artifacts/datasets/base5_v2_live_lidar1_camera0/gpu_live_predict_smoke.json` | 새 live 데이터41장 로딩 및 단일 GPU loss/decode finite |

Live에서는 첫 ScenarioLoad 뒤 MR rosbridge 연결이 모두 끊겼다. network=false 재시도도
pose 확인에 실패했다. Player.log의 ScenarioLoad 완료/인터페이스 disconnect와 ROS 미수신을
확인했지만 단일 근본 원인은 확정하지 않았다. 이후 연결이 복구되어 pilot을 재개했다.
Network 재로드 직후 service duplicate overwrite 로그도 확인했다.5초 timeout,
재광고 안정화 대기 및 idempotent mode/gear 요청1회 재시도로 연결 문제를 완화했다.
검증 응답을 생략하지 않으며 최종 통신 실패는 중단한다. MR 재시작은 하지 않았다.

v15 전체는 **OFFLINE_PASS이지 전수 LIVE_PASS가 아니다.** v11/v13/v14는 수정 전/검증 실패 중간 산출물이므로 캡처 후보로 쓰지 않는다. 43개 asset은 아직 캡처 기반 geometry
reference가 없어 보수적 크기를 사용한다. 지면/애니메이션/실제 mesh·collider, 근접 partial-view,
원거리 가시성은 이 검사로 인증하지 못한다. 인접 link 사이 시야 중복도 가능하므로
현재 train/val split을 최종 generalization test로 주장하지 않는다.

회피 소량 live 판정: `concrete-barrier`와 `OBJ_Hyundai_HD65`의 첫 초근접 scene은
상태 토픽과 annotation에 객체가 없었고 Player.log에 `cannot be placed here`가 있었다.
객체 위치/yaw/asset을 유지하고 Ego만 외측 이동하니 두 경우 모두 복구됐다.
따라서 이번 누락은 단순 센서 사각이나 항상 실패하는 asset이라는 결론이 아니라
**근접 배치에 따른 MORAI 로드 거부**로 판별한다. 구체적인 내부 collider 규칙은 미확정이다.
특히 HD65 fallback 추정 size5×2×1.8m와 actual annotation6.557×2.517×2.457m가 달라
계획한 간격을 실제 간격으로 간주하면 안 된다. 임의 자산 제거나 GT 크기 축소로 해결하지 않는다.
범용 최소0.3m도 actual collider 안전 인증이 아니며, unknown geometry의 calibration이 필요하다.
총12장 raw 저장을 성공했지만 지면/CollisionData 증거가 없어 **accepted 학습 후보는0**이다.
이 단계는 초근접 캡처 가능성/배치 실패 원인 확인이지 OD 개선·회피 성공률 검증이 아니다.

## 다음 순서

1. [동기 캡처 런북](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md)대로 연결/timeout gate를 유지한다.
2. 완료된48장 및 수정 후 재캡처: 파일 완결, 정지/동작, UID/component, 실제 GT footprint·거리·visibility 검사.
   새 캡처는 기록한 LiDAR→Ego calibration으로 GT center를 옮겨 거리 bin을 확인한다
   (0.5 m 허용 오차). Calibration 없는 과거 fixture는 LiDAR 거리/2.5 m의 제한적 검사다.
3. 실패 quota를 수정/보충하고 annotation 지면 검사 및 미검증 asset을 확인한다.
   동적 미션은 앞/뒤 횡단보도·도로 횡단의 명시적 geometry quota가 아직 부족하다.
   현재는 validator anchor 주위 독립 배치와8m actor 이동 segment 검사다.
4. Zero-point camera 사례로 positive 정책을 비교하고4-class/runtime 여부를 확정한다.
5. 본 수집 승인 후10,000 **accepted** frame을 모은다. 이번 생성10K가 accepted10K를 보장하지 않는다.
6. Base5-only from-scratch 학습 → 동일 mission held-out bag의 Detection/Tracking/Map-CV.

### Live에서 발견한 수정과 GT 정책 비교

- SUV/Sedan 등 차량 annotation subtype을 Vehicle로 정규화했다. 미지 class는 조용히
  Obstacle로 처리하지 않는다. 실패한 캡처도 raw/provenance를 남긴다.
- v9의 dummy는 `static`을 `standing`과 다르게 처리한 분기 때문에 speed8로 움직였다.
  v10에서 speed0/non-waypoint로 고쳤고 실제 재캡처가 통과했다.
- 다른5개 근접 동적 scene은 대기 중 이동으로 거리 bin을 벗어났다. 실제 GT를 원하는
  위치로 조작하거나 허용 오차를 늘리지 않고 capture timing을 바꿔 재검증했다.
  고정1.2초 대기 대신0.05 m 실제 이동을 확인한 즉시 캡처하고 제어 후 대기를 줄여
  1/5장이 통과했다. 평균 약4.864초/장이지만 나머지4장은 여전히 실패한다.
  **본 수집 gate 미통과**다. 이후에는 actor 시작 위치와 예상 capture 시점의 목표 위치를
  구분하고, 시작/종료 footprint 비충돌을 유지하면서 짧은 near bin의 phase를 맞춰야 한다.
  이벤트 재연결 지연까지 고려하여 예상시점만으로 성공 처리하지 않고 실제 GT bin을 확인한다.
- 후방80–99m 차량1장은 annotation/센서 evidence 누락으로 검토다. 이를 empty negative로
  쓰지 않고 가시성이 확보되는 별도 배치로 보충해야 한다.
- Camera pixel gate는 원본 pixel이 아니라 실제896-wide resize 후320 bottom-crop에서
  계산하도록 고쳤고 모든48장을 원본을 보존한 별도 audit 버전으로 다시 검사했다.

| 변환 후보 | 유지 scene | 유지 GT | 내용 |
|---|---:|---:|---|
| LiDAR≥5 | 33 | 42 | pilot Animal positive0, sparse 외관 누락 |
| LiDAR≥1 | 40 | 52 | Animal1, camera-only component 제외 |
| LiDAR≥1 OR camera-visible | 41 | 55 | Animal3, 실제 zero-point camera 사례 보존 |

이 비교는 accepted 후보의 primary가 해당 policy에도 보이는 경우만 변환한다.
전체48장의 raw GT는66component다. 이 pilot 선택은 train41/val0이므로 validation accuracy
목적으로 사용할 수 없다. 다음 pilot selector는 split도 coverage tag에 포함한다.

실측48장 raw는 약221 MiB, 평균5.576초/장·p50 5.381초였다. 단순10K 외삽은
약45 GiB/15.5시간이며 실패 보충·추가 metadata·환경 부하를 포함하지 않는다.
따라서 현재 single-frame load 경로에서10K 본 수집이2–3시간이라고 예상하면 안 된다.

전체 수집시간/용량을 pilot 외삽만으로 보장하지 않는다. 10K 파일을 MR 시나리오 GUI에
한꺼번에 넣지 않는다. Runner가 소량 runtime 파일을 설치하고 자신이 만든 파일만 회수한다.
