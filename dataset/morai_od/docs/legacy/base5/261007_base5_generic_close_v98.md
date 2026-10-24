# Base5 범용 좌우 근접 보강 — v98

2026-10-07. 구현·대표 검증 완료. 최종5K 본수집 승인 또는 근접 문제 전체 해결은 아님.

## 배치 정책

- 범용 Obstacle 및 정적 Vehicle 중 기존 center 거리 bin 상한≤15m인 셀의30%를
  BBox 경계 간격 기반 alongside 관측으로 바꾼다. 총175셀(Obstacle172/정적Vehicle3).
  미션 셀·기존 원거리 셀·primary asset·population mode/예산은 유지한다.
- 경계 간격 요청0.3–0.5m 50%,0.5–1m 30%,1–2m 20%, 좌우 오차 최대1셀로 층화한다.
  측정 geometry는 최소0.3m, 미측정 geometry는 최소1m의 기존 보수적 여유를 유지한다.
  요청값·effective 계획값·실제 annotation 간격을 구분한다.
- 기존 정적 회피 observation solver를 재사용하지만 범용에서는 Ego XY를 도로 anchor에
  유지하고 객체를 옆으로 옮긴다. 범용 Ego를 도로 밖 가상 회피 위치에 놓지 않는다.
  이 부분은 yaw/lateral 일반 랜덤 quota의 예외이며 Ego 실제 lateral offset metadata는0이다.
- 객체 위치는 도로변이 될 수 있다. 접지·CollisionData·센서 가시성·GT 검사를 통과해야
  positive로 편입한다. 모든 mesh/collider 및 도로변 지면을 사전 인증한 것이 아니다.
- 측방 관측 quota를 늘리는 변경이므로 과거 role/거리 bin은 별도 원본 spec 필드에 보존한다.
  family 총량과 원거리 coverage는 보존하지만 전체 방향 비율까지 불변인 것은 아니다.

## 실제 근거와 수정 루프

비추적 근거: `/home/luv/morai_cap/base5_v2_261007/`.

- 175/175 조건 생성 가능, 최대재시도4회/평균0.434회. 전체5K JSON 생성은 아니다.
- `generic_close_v97_offline64`: 64 unique payload/전수 OFFLINE_PASS.
- v96 초기 방식의 Concrete_Barricade02 4actor 혼합은 ground QA 후 통과했다.
  YellowBarrel은 실제 약0.333m 간격에서 primary 센서 근거가 없어 review였고,
  대체 배치는 Ego drift/지면 review, 다음 시도는 preflight 실패로 halt했다.
  Steel_Barricade 조건도 새 CollisionData Ego pose를 확인하지 못해 촬영 전에 halt했다.
  사후 Ego 위치가 요청에서 XY 약0.89m 달랐으므로 단순 토픽 누락이라고 단정하지 않는다.
  도로 가장자리 배치 위험을 줄이기 위해 Ego 도로 anchor 유지 방식으로 보완했다.
  정확한 drift의 물리 원인(지면/curb/충돌 등)은 미확정이며 안전 gate를 완화하지 않았다.
- `generic_close_v97_live`: Concrete_Barricade02 4actor, 1/1·14.73초.
  실제 primary annotation 간격0.43274m,4/4 actor 가시·GT4,접촉0·Park 확인.
- `generic_close_v98_right_live`: Steel_Barricade 2actor,1/1·6.71초.
  실제 primary annotation 간격0.44690m,2/2 actor 가시·GT2(camera-only component1).
  접촉0·Park 확인. 완료resume 추가 캡처 없이 검증 성공.
- 보완 후 좌우 대표2장 모두통과/CPU2장·Obstacle GT6개/제외GT0, raw 보존.
- 회귀215 tests PASS(최종 Ego offset 메타데이터는 별도3개 근접 단위검증으로 재확인).
- v96 CargoBox 별도1/1은 과거 배치 방식의 근거로 보존하며 최종 좌우2장에 합산하지 않는다.
  halted 세션을 강제resume하지 않았으며 매번 새output을 사용했다.

## 남은 일

- 작은 낮은 객체의 초근접 센서 blind spot이 해결된 것은 아니다. 보이지 않는 primary를
  억지 positive로 넣지 않는다. 실패/부족분 보완에서 가시 조건 확보 여부를 확인한다.
- 범용 정적 차량은3셀뿐이므로 차량 초근접 충분성을 주장하지 않는다.
  고주로 추월·로터리 앞뒤 밀집 패밀리의 차량 보강이 다음 우선순위다.
- 정적 미션 기존 approach/entry/alongside/clearing 정책은 유지했다. 이번 대표 live는
  범용 alongside이며 미션의 모든 단계나 실제 회피 주행 성능을 인증하지 않는다.
- 그 뒤 희귀 자산 유효 coverage 및 GT 대표 검증, 새5K manifest 전수검사로 진행한다.
  본5K 캡처·학습·production E25/tracker/planning 변경은 미실행이다.
