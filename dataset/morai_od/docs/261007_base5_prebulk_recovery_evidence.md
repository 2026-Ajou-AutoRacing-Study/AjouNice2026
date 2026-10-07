# Base5 본수집 전 recovery 검증 — EVIDENCE

2026-10-07. 본10K 미재개, production/학습 미변경. 원본 센서 GT는 수정하지 않는다.
실측 파일은 `/home/luv/morai_cap/base5_v2_261007` 아래에 보존한다.

## 먼 복합 객체

`remaining_live_v78`/`remaining_live_v79`에서 약49m BabyStroller1 혼합 장면을
원본 raw 재검사로 통과시켰다. 이전 single-box 전용 witness 대신, 알려진 복합 자산의
고유 rigid child(유모차 Obstacle, 탑승자 자산의 Vehicle)를 좌표계 연결 anchor로 사용한다.
모든 child의 class/size/center/rotation 일대일 대응과 전체 footprint support도 검사한다.
여러 GT를 합치거나 baby GT를 삭제하지 않는다. person+pet에는 이 방식을 적용하지 않는다.

BabyStroller1의 adult/stroller/baby 3개 center 대응 잔차는 최대 약2.88μm였다.
해당 raw는 세 component 각각 LiDAR1 point, PE_Drum5 points였다. 이는 현재 1-point
positive 계약의 통과이며 먼 객체 검출 성능을 인증한 결과는 아니다.

근접 calibration의 mapped-road 관찰 위치 탐색만8→12m로 확장했다. 실제 학습 장면의
actor 위치/거리 bin/GT/ROI를 바꾸지 않는다. 지형 높이 호환성과 실측 평면·전체 footprint
검사는 유지한다. witness raw hash 손상은 placement review가 아닌 fatal 오류다.

## Dog2 혼합 장면의 단계별 실패와 보완

- `remaining_live_v79`: 2셀 중1통과/1review,60.65초. Dog2 near에서
  annotation-ground gap+0.4038m를 측정했지만, 낮춰 재촬영한 원래 장면은 가시성이 부족했다.
- `remaining_campaign_v80`: quota campaign124.47초,1통과/1deficit.
  동일 quota 재배치 후보도 근접 지면 관찰 위치를 찾지 못하는 경우가 반복됐다.
- `dog_ground_campaign_v81`: 같은 Dog2 quota1/1,40.68초,deficit0.
  missing-ground 보충 후보는 사전에 모든 정적 actor의 mapped-road 관찰 위치가
  가능한지 검사한다. 실제 LiDAR 평면/가시성/충돌 검사는 이후 그대로 수행한다.

Near의 실제 pose가 원래 장면과 일치하고 실패가 above-ground만일 때만 측정된
downward input-Z 후보를 원래 장면에 적용한다. 누적75cm 한도, 원본 미수정,
변경 payload의 실제 재촬영·새 ground witness를 요구한다. 가시성 면제는 없다.
한 cell의 내부 보완은6단계, campaign quota 보충은 기본2round로 bounded다.
이를 모든 Dog2 위치의 접지 보장이나 모든122자산 인증으로 확대 해석하지 않는다.

## Campaign/변환 연결

원본 raw 재검사 QA view는 ROS session이 아니다. 이를 가상의 종료 Park/session_contract로
검증하던 연결 오류를 수정했다. 대신 원본 metadata/audit/scenario/raw hash와 재계산 audit를
확인하고 실제 원본/near session의 Park·충돌 근거를 별도로 검증한다.
`remaining_campaign_v80`의 far stroller가 실제 quota campaign에서 이 경로로 편입됐다.

32-cell capacity deck은 current generator로 재생성하는 opt-in을 추가했다.
원래 spec/primary asset/split/quota를 유지하고 현재 natural-target 정책을 사용한다.
10K 전체를 새로 만들거나 캡처하지 않는다. 최종 묶음·pause/resume·CPU 변환 결과는
후속 검증 완료 후 아래에 기록한다.

## v82 다중 actor 연쇄 QA 재현·수정

`prebulk_batch_v81`은6장 승인 뒤 후방 K5+Sonata 장면에서 `Re-audit raw changed`로
117.06초에 halt했다. 두 near session 종료 Park는 모두 확인됐다. 두 번째 actor의
ground witness를 연결할 때 첫 QA view의 `reaudit_result.json`까지 raw 목록으로
복사했으나, 다음 QA 결과도 같은 파일명에 저장해 raw hash 비교가 실패했다.
센서 변경이나 MORAI 충돌이 아닌 QA 산출물 분류 오류다.

`reaudit_result.json`을 raw 목록/하드링크 복사에서 제외하고 source가 QA view면
이전 QA 근거를 재귀 검증한다. 이전 sensor/raw·metadata/audit hash 검사와 실제
session safety는 유지한다. 이전 결과 파일 불변 및 이전 status 위변조 거부를
단위 재현했다. 최종190 tests PASS. halted 출력은 수정/resume하지 않고 보존하며,
새 `prebulk_batch_v82`에서32셀 batch·8셀 pause·resume 검증을 시작했다.

## v82 최종 batch / v84 마지막 부족분

- v82:32셀 처리,31승인/1deficit,누적 active wall486.304초.
  8/8 승인 뒤125.823초에 정상 pause했고 resume했다. 앞8셀의20개 ledger/QA 결과
  hash 불변이다. 완료resume14.256초는 검증 전용이며2,332개 raw/ledger/QA 파일의
  수/hash와 누적시간 불변,추가촬영0을 확인했다.
- 실제 시도68회(학습 승인31장과 별개),raw 재검사18회.56개 child 종료 Park 및
  boundary 검증,unsafe0,event timeout0,살아남은 service worker0이다.
  EventCmd handler None 응답7회는 재시도 회복됐으며 오류0이라고 쓰지 않는다.
- CPU 변환31장(train25/val6/test0),GT44개(Vehicle20/Pedestrian7/Obstacle15/Animal2).
  카메라4장/4-feature LiDAR/유한 calibration/7D BBox sanity 오류0,empty2장.
  camera-only Obstacle3개도 현재 정책대로 유지했다. 학습 forward/성능 검증은 아니다.
- 남은 셀은15–30m/front/Ped_Pet1 두 객체의 동적 캡처다.
  v83의3-round 보충도41.57초/0승인으로 끝났다. 실제 빈 지형의 static target plane이
  없었으며, 검사 완화나 자산 제외로 처리하지 않았다.
- v84에서는 원래 빈 지형이 sparse할 때만 각 moving actor 목표 위치 근처의 mapped-road
  빈 지형을 추가 관찰한다. 최대3개 추가 witness/총4개 session으로 제한하고,
  동일 source/hash/empty/calibration/Park/충돌과 전체 footprint support를 요구한다.
  motion target과 출발점을 구분한다. 전체 경로의 실측 지형/10cm step 한도는 유지한다.
  첫 terrain+정적 reference+추가 near2+최종 capture의5촬영으로 같은 셀1/1,
  37.505초 통과. person/animal GT4개(LiDAR21/2/10/6point)를 보존했다.
  primary 실제 이동 약1.107m/s이며 모든 motion review는 비어 있다.

v85 마지막8조건 회귀와 current10K **오프라인 scenario 준비**를 진행 중이다.
본10K 센서 캡처는 승인/시작하지 않았다. v82(31)+v84(1)를 한 버전에서32/32로
실행한 것처럼 표기하지 않는다. 구 v82를 새 capture 코드로 강제resume하지 않는다.
