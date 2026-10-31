# Base5 혼합·복합·동적 지면 검증 — v77 EVIDENCE

2026-10-07. 본10K 재개가 아닌 bounded live 검증이다. Production/GT 정의/ROI는 변경하지 않았다.
근거 루트는 `/home/luv/morai_cap/base5_v2_261007`이며 Git에 센서 원본을 넣지 않는다.

## 구현

- 원거리 지면 보완은 primary뿐 아니라 실패한 static secondary UID도 대상으로 한다.
  `prepare_base5_static_height_reference.py --target-uid`는 해당 actor payload를 그대로 유지하고,
  가까운 mapped-road Ego에서 단독 calibration을 만든다. 나머지 객체를 원본에서 삭제하지 않는다.
- 실제 pose/size/정지 여부/hash/full-relative R/T/전체 footprint 검사는 기존과 같다.
  결과는 원본 raw를 공유하는 별도 QA view다. GT·sensor bytes는 바꾸지 않는다.
- 여러 secondary를 순차 검증할 때 기존 witness 목록을 보존한다. 같은 UID만 새 proof로 교체한다.
- 실제 annotation이 여러 component인 actor에는 single-rigid 방식으로 인증하지 않는다.
  component 수를 확인해 불가능한 rigid calibration 촬영을 피한다. 빈 지형 보완/동일 quota
  재배치로도 확인하지 못하면 review를 유지한다. 복합 객체를 하나의 박스로 합치지 않는다.
- 구 `prepare_base5_readiness_pilot.py`가 natural-target source를 one-way-slow-v1으로
  덮어쓰던 문제를 수정했다. 현재 generator의 natural-target/appearance 계약과 source 준비
  budget을 사용한다. Quality loop도 target_position 없는 구형 이동 deck을 calibration 전에 거부한다.
- 원본 QA 재검사 resume의 tuple/list 비교 오류도 수정했다. Shapely polygon tuple을
  JSON 값으로 정규화한 뒤 저장 audit와 비교하며 실제 값 변경은 허용하지 않는다.
  이미 만들어진 QA view의 source/reference/raw/재검사 불일치는 RuntimeError로 중단한다.

## 실제 검사

| 세션 | 결과 | 부모 전체 시간 | 해석 |
| --- | --- | ---: | --- |
| mixed_live_v73 | 0/4, ground review4 | 105.29초 | 초기 탐색; 복합/보조 actor의 미검증을 드러냄 |
| mixed_targeted_live_v74 | 2/4, review2 | 86.12초 | 보조 차량·cycle 혼합 해결; 확정 코드의 순수 시간 A/B가 아님 |
| complex_near_live_v75 | 2/2 | 12.30초 | 가까운 Bike3·BabyStroller2, local ground로 확인 |
| dynamic_live_v75 | halt | 7.00초 | 구 도구 target_position 누락; empty calibration1회만, 이동 payload 미로드 |
| dynamic_live_v76 | 2/2 | 47.14초 | 지형2+정적 기준2+동적2의 실제6촬영, 준비/감사 포함 |
| mixed_final_live_v77 | 2/2 | 35.61초 | 최종 runtime, 근접 witness2회/원본 재검사2회, 한 quota는 동일cell 재배치 |

초기 탐색 중 코드가 보완됐으므로 v73/v74를 고정 runtime의 속도 개선율로 사용하지 않는다.
모든 성공은 annotation-ground proxy gate이며 정확한 mesh contact/collider 인증은 아니다.

`mixed_final_live_v76`의 최초2/2 후 완료resume에서 tuple/list 비교 오류로 불필요한 보충
촬영이 발생했다(약15.59초, resume 후1/2). 원래 sensor/witness hash는 유지됐고 이전 QA
view도 보존했다. 이를 정상resume로 계산하지 않는다. 수정 후 새 v77에서는 완료resume
약1.104초/추가 센서파일0/센서·Instance·종료Park hash 불변/선택2/2 유지로 재확인했다.
v77 CPU 변환2장/GT5(Vehicle1/Obstacle4), calibration 학습 편입0이다.
가까운 PE_Drum 약5.57m+XC90 약44.95m, 근접 cycle+보조 cycle 약38.75m의 혼합을
확인했다. 두 번째는 source의 primary/quota/split을 유지한 재배치이며 모든 secondary까지
최초 배치와 동일했다고 주장하지 않는다.
단위회귀183 tests PASS, py_compile/diff 공백 검사 PASS.

### 가까운 복합 객체

- Bike3: 2component(자전거 Vehicle78point, 사람 Pedestrian148point), ground gap 약+7.1mm.
- BabyStroller2: 3component(성인113point, 유모차17point, 아기0point), gap 약−1.1mm.
- CPU 변환2장/GT4: Vehicle1/Pedestrian2/Obstacle1. 0point 아기는 raw에 남고 현재
  lidar1_camera0 정책의 positive에서만 제외됐다. invisible 아기를 없앤 raw가 아니다.
- CollisionData peer 접촉0, 종료 Park 확인. 실제 배치 거리 약5.8m/우측3.8m.

### 실제 이동

- Man7/Woman5 달리기 관측 속도 약2.150/2.149m/s, Dog1 약1.193m/s.
  receive-time 위치 차분 진단이지 학습 velocity GT가 아니다.
- 수신 창 최대 Z step: Man7 0.353mm, Woman5 17.475mm, Dog1 1.198mm.
  10cm step gate 이내이며 각 actor의 지면 검사도 별도로 통과했다.
- annotation-bottom ground gap: 약−5.2/−3.0/−5.6cm. 지형 이동과 부유를 혼동하지 않는다.
- 두 동적 장면의 CollisionData peer 접촉0, Ego Park/정지 유지.
- CPU 변환2장/GT3(Pedestrian2/Animal1), calibration4장은 학습 제외.

## 남은 조건과 다음 작업

- 먼 BabyStroller1 복합 장면은 지면 support 미확인으로 review다. 근접 stroller 성공이
  원거리 복합 actor까지 인증한다는 뜻은 아니다.
- Dog2 두 개 혼합은 above-ground+미측정 plane이 남아 review다. input offset 후보만으로
  통과시키지 않는다. 실패 raw/보완 시도는 보존한다.
- 원거리 secondary2조건 최종 검증·완료resume는 통과했다. 전체 캠페인의 혼합 ground
  deficit 비율과 실제 재배치 비용 분포는 아직 추가 표본이 필요하다.
- 동적 평균23.57초/장, 가까운 복합 평균6.15초/장은 각2조건의 실측이지10K 대표 시간이다.
  원거리/혼합 보완 및 실패 보충 비율이 아직 부족하므로18.7h 또는 새 단일 ETA를 확정하지 않는다.
- 본10K/학습은 미재개다. 원거리 복합 ground witness와 혼합 동물 높이 보완이 우선이다.
