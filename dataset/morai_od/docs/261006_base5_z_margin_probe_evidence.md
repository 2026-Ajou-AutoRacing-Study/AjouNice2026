# Base5 입력 Z 여유·지면 보정 대조 — EVIDENCE v30

2026-10-06. 초기 지면 겹침 → 위로 밀려남 가설을 실제 MR에서 시험했다.
Production detector/tracker/planning/localization, raw GT, 본10K는 변경하지 않았다.

## 결론

이번 세 자산·한 위치에서는 **2/5/10cm 상향 여유가 부유를 해결하지 않았다.**
Dog2와 Ped_Pet1은 입력 Z 증가에 거의1:1로 최종 높이가 증가했다.
Scooter1은 정지일 때 지면에 내려앉지만 waypoint 이동일 때 runtime position.z가
입력 Z보다약0.340m 높게 유지됐다. 작은 상향 여유에도 이 차이는 유지됐다.
안전 격자 크기로 튕겨 올라간다는 근거는 얻지 못했다.

신뢰할 수 있는 로컬 LiDAR 지면과 annotation 하단을 비교한 다음 **입력 scenario와
waypoint Z를 함께 낮추고 새로 촬영**하니 보정 후24조건 전부 기존 품질 gate를 통과했다.
측정량은 그 위치/자산/모드 전용이며 전역 상수로 적용하지 않는다.
맞는 지면 높이를 구한 후0–2cm 여유를 시험할 수 있지만, 기존 입력 Z에 일괄2cm를
추가하는 정책은 채택하지 않는다. 일반 수집은 v29의 관측·보정·재검증 절차를 유지한다.

## 실제 수행

동일 Ego pose와 actor 목표 위치/속도/방향에서 Dog2, Ped_Pet1, Scooter1의 정지/이동을
비교했다. pos.z, initPos/standardPos.y, waypoint.pos.y, metadata/target Z를 함께 바꿨다.
센서·network는 기존 동기 seven-interface profile 그대로다.

| 단계 | 시도 | raw | 결과 |
|---|---:|---:|---|
| 기존 입력 +0/2/5/10cm | 24 | 22 | 정지 Scooter1 4장만 전체 gate 통과, 나머지18장 부유 review |
| 실측 지면 보정 후 +0/2/5/10cm | 24 | 24 | 24/24 전체 gate 통과 |
| 보정 기준에서 −5/−10cm | 12 | 11 | 큰 상향 튐 미관측; 의도적 깊이 대조라 학습 선택 금지 |
| 합계 | 60 | 57 | 목표도착 미확인3회는 캡처 차단, 실패 이력 보존 |

모든57 raw에서 Ego-peer CollisionData 비충돌을 확인했고 세 세션 모두 종료Park을
확인했다. 사전Park은약11.7ms 성공. 종료마다 첫 요청timeout/재시도 성공이 있었으므로
EventCmd 지연 원인이 해결됐다는 뜻은 아니다. 안전 기록 변경·실패 raw 삭제 없음.

### 대표 수치: annotation 하단–지면 gap

| 자산/모드 | 기존 입력 | 지면 보정 후 여유0 | 보정 후 여유2cm |
|---|---:|---:|---:|
| Dog2 정지 | +28.63cm | −2.93cm | −0.93cm |
| Ped_Pet1 이동 | +21.62cm | −2.06cm | −0.95cm |
| Scooter1 이동 | +59.74cm | −2.85cm | −0.90cm |

Dog2 이동의 기존여유0은 목표도착을 놓쳤으므로 결과를 만들지 않았다.
여유2/5/10cm에서 gap은+28.14/+31.12/+36.01cm로 증가했다.
지면 보정 후 이동4조건은 모두 정상 높이로 캡처됐다.
보정 후 여유10cm는 gap약+7cm까지 올라간다. 허용 gate 통과가 최상의 지면 접촉은
아니므로 이를 기본값으로 삼지 않는다.

여유0 대조의 입력 하향량: Dog2 정지/이동 −31.63/−29.14cm,
Ped_Pet1 정지/이동 −30.14/−24.62cm, Scooter1 이동 −62.74cm.
Scooter1 정지는 수정 없이 gap−4.93cm였다. 지면 오차·OBB 마진·자세 차이가 있으므로
이 수치를 공식 pivot offset이나 전 위치의 terrain correction으로 등록하지 않았다.

### 아래쪽 배치·시간축 판별과 제한

보정 기준에서−5/−10cm 낮춘 Dog2/Ped_Pet1/이동Scooter1은 annotation gap이
대략−8/−13cm로 내려갔다. 정지Scooter1은 같은 지면 높이에 머물렀다.
이 대조에서도 큰 상승은 관측되지 않았다. 정지Scooter1의 출력은 입력에 종속되지
않으므로 입력−10cm를 실제 mesh 침투10cm라고 해석하면 안 된다.
Annotation 하단은 collider/발바닥의 정확한 접촉점이 아니며 collider–지면 contact를
별도로 읽은 실험도 아니다. CollisionData는 Ego와 peer의 충돌 증거로만 사용했다.

장면 pose 확인 이후 인접수신≤0.2초에서57장의 최대 Z step은약1.91e−6m였다.
이번 부유는 확인 이후50cm 튀는 패턴보다 **이미 높게 유지되는 배치/이동 모드**에
가깝다. 최초 publisher 전에 발생하는 물리 처리는 관측하지 못한다.

첫24조건은 기존probe UID8500을 재사용했다. 이전 객체의 늦은 메시지가 섞일 수
있으므로 pose-confirmation 이전 메시지는 물리 튐 근거에서 제외했다.
후속36조건은 장면별 UID를 분리했다. 이전 raw는 그대로 보존했다.
동일표본 반복 통계 검정, 전 자산/경사/지형 인증, 내부 물리 코드 확인은 아니다.

## 구현·검증 및 다음

- `prepare_base5_z_margin_probe.py`: 최대64조건, 높이 필드 동기화·고유UID.
  안전 종료된 측정표본만 bounded 하향 대조에 사용하고 source scenario SHA를 확인한다.
- `report_base5_z_margin_probe.py`: scene-confirmed Z trace, 입력응답, 지면 gap,
  RGB crop contact sheet/trace plot. 실패도 report에 남긴다.
- 새6개 테스트 포함 Base5 총82개 PASS, Python compile/diff whitespace PASS.
- 실험 전용이다. 의도적으로 낮춘 대조군을 일반 selector로 학습에 넣지 않는다.
  세션 안전 gate·학습 GT를 완화하지 않았다.

원본은 `/home/luv/morai_cap/base5_v2_261006/`의 `z_margin_park_preflight_v30.json`,
`z_margin_{deck,capture,report}_v30/`, `z_margin_ground_{deck,capture,report}_v30/`,
`z_margin_below_{deck,capture,report}_v30/`에 있다. 각 report의 report.json/contact_sheet.png/
z_response_traces.png와 개별 RGB crop, 각 capture의 scenario/metadata/telemetry/annotation/
CollisionData 원본을 보존했다. Git에는 코드·문서만 반영한다.

다음은 남았던 근접·혼합 셀(Deer1/BabyStroller3/Scooter1)의 소량 마무리다.
새 위치에서도 재검증하며 이번 scalar를 복사하지 않는다. 이후 MR 종료 후 GPU 속도 측정.
본10K/본학습은 아직 시작하지 않았다.
