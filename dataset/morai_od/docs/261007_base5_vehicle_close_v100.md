# Base5 v100 차량 미션 근접 — EVIDENCE

2026-10-07. 독립 Sunny11시 5K의 고주로·로터리 근접 관측 보강이다.
본수집/학습/실제 추월 제어를 수행한 결과는 아니다.

## 구현

- `base5_vehicle_close.py`는 기존 미션 근거리 셀 일부만 교체한다.
  고주로66셀, 로터리67셀이다. primary 자산·객체 수·topology를 유지한다.
  원거리 quota와 범용 고주로의 despawn 이후 범위는 변경하지 않는다.
- 고주로3차로: 앞옆/측방/측후방 각12셀. 나머지 구간 앞/뒤 밀집 각15셀.
  로터리 앞34/뒤33셀. 좌우는 균형 deck을 사용한다.
- NPC pivot은 실제 MGeo NPC 경로 위에 둔다. 고주로 미션은 샘플 spawn/despawn
  corridor 안으로 제한한다. NPC 임의 yaw·횡 jitter는 넣지 않는다.
  Ego의 기존 yaw·횡 offset 범위는 유지한다.
- 후보의 annotation footprint 경계 간격은0.3–2.5m이다. 전수 생성된 primary
  실제 계산값은0.308–2.030m였다. 중심거리0–30m는 차량 크기 차이를 허용하는
  후보 범위이며 30m 객체를 초근접으로 집계하는 정책이 아니다.
- v99 전수 검사에서22셀의 간격 witness 불일치를 발견했다. scenario yaw 저장의
  소수3자리 반올림 전후를 혼용한 문제였다. 저장되는 yaw로 후보 footprint를
  계산하도록 수정했고, tolerance를 완화하지 않았다. v99는 실패 근거로 보존한다.
- campaign hash에 차량 근접/회피 geometry helper를 포함했다. 변경 전 campaign을
  새 코드로 강제 resume하지 않는다.

## 검증

- 전133셀 bounded 생성 가능. 5K 전체 생성 재시도 최대149회/평균1.366회.
- `/home/luv/morai_cap/base5_v2_261007/final_5k_v100/manifest.json`:
  5,000 unique payload, train4,397/val603/test0, 전수 `OFFLINE_PASS`, 오류0.
- live root: `/home/luv/morai_cap/base5_v2_261007/vehicle_close_v100_live7`.
  indices39/200/375/418/671/723/855, 총7/7 승인, 부족0,85.58초.
  두 로터리와 다섯 고주로 phase를 각각 확인했다.
- 원본 sensor annotation의 primary footprint 간격은0.508–2.002m였다.
  13개 배치 actor 중12개 positive ROI GT가 유지됐다. Porter의 secondary 한 개는
  가시성 근거가 없어 제외됐다. 이 정상 제외를 detector 검출 실패라고 해석하지 않는다.
- CPU 변환7장/Vehicle GT12개. calibration/review/failed 장면은 편입하지 않았다.
  rigid-ground 재감사는 원본 센서 파일을 재사용하며 실제 충돌·Park 근거를
  source capture까지 추적한다. re-audit ledger의 nullable collision 필드만 보고
  충돌 검증을 생략했다고 해석하지 않는다.

## 한계

이것은 정지 Ego에서 보이는 근접 외관을 확보하는 정책이다. 실제 100km/h 추월,
모든 차종/지형의 무충돌·가시성·모델 정확도 인증은 아니다. 최종 캡처마다 접지,
CollisionData, Ego pose, Park, GT 가시성 검사를 계속 수행한다. 센서 blind spot은
억지 positive GT로 만들지 않는다. Production E25/threshold/tracker는 변경하지 않았다.
