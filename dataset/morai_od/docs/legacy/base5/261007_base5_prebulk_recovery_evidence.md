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

## 최종 v85/v86 및10K 준비

- v85:걷기·달리기·먼 유모차·Ped_Pet1·Deer1+Dog2·탑승자·후방 다중 차량·WoodBox
  8조건8/8,164.323초.22개 child 종료 Park/unsafe0,CPU8장/GT18개/sanity오류0.
  완료resume4.172초는 검증 전용이다.
- 마지막 코드 점검에서 animated review→내부 placement replacement→지형 준비 없이
  recapture할 수 있는 경로를 제거했다. review 근거를 부족분으로 남기고 outer quota
  round가 같은 spec/primary/split의 새 배치를 새 terrain/static calibration부터 준비한다.
  통신/Park/근거 손상 오류를 quality deficit로 덮지 않는다. seed moving review에서
  내부 replacement/추가 Popen이 없음을 단위 재현했다.
- 최종 v86 Ped_Pet1·Deer1+Dog2 동적2/2,60.568초,CPU2장/GT6개.
  캡처 요청 직전0.5초 이후 연속 관측의 최대Z step은Ped_Pet1 약0.73mm,
  Deer1 약0.45mm,Dog2 약1.59mm였다. Dog2 전체 준비 trace에는 요청5.6초 이전
  약0.348m 차이도 있어 전체 준비 과정의 Z jump0이라고 주장하지 않는다.
  UID 재사용/로드 전환을 포함한 준비 trace와 캡처 직전 안정 구간을 구분하며,
  실제 캡처의 ground/모든 actor motion/Park 검사는 통과했다.
- 마지막 단위회귀198 tests PASS.
- current10K는 `scenarios_10k_v85/manifest.json`에 생성했다.
  highway3000/roundabout700/other_vehicle300/obstacle3000/pedestrian_related2500/background500,
  unique payload10000,train8614/val1386/test0다.
  최초 offline_audit의1569개 오류는 검사기가 자산별 saved-height initializer 대신
  차량 공통+0.36m만 기대한 불일치였다. 원본 F2·MGeo에서 후보를 독립 재계산해
  metadata와 비교하고 실제 scenario XY/Z도 검사하도록 수정했다.
  `offline_audit_current.json`:OFFLINE_PASS/오류0. 최초 실패 보고서도 보존한다.
- `scenarios_10k_ordered_v86/manifest.json`은 모든 source row를 그대로 유지하고
  정적/정지8354→동적1646 순서만 변경한 최종 운용 후보다. 전수OFFLINE_PASS/오류0,
  같은 manifest의 첫4셀 실제4/4(60.408초)도 통과했다. 동적 객체를 정지시키거나
  별도 quota를 빼서 속도를 높인 것이 아니다. 정적/동적이 섞여 정적 셀도 isolated
  motion 준비 경로로 보내던 batch overhead를 줄이려는 순서 개선이다.
  순서 변경의 실제10K 단축률은 아직 측정하지 않았다.

본10K 센서 캡처는 승인/시작하지 않았다. v82(31)+v84(1)를 한 버전에서32/32로
실행한 것처럼 표기하지 않는다. 구 v82를 새 capture 코드로 강제resume하지 않는다.
전수 오프라인 검사도 전수 live 인증이 아니며 각 scene의 실제 ground/visibility/
motion/CollisionData/Park 검사와 부족분 분리는 본수집에서도 계속 필요하다.

v82 parent wall/승인31장으로 단순외삽하면10K 약43.576시간이다.
실제 capture attempt의 raw bytes/승인31장으로는 약103.558GiB다.
이는 QA/calibration/실패 시도를 포함하지만 coverage-biased 표본이며 최종v86의
family-weighted ETA나 보장값이 아니다. 성공 capture의 duration만으로 계산한14–18시간을
campaign ETA로 쓰지 않는다. 구18.7시간은 현재 정책 예상에서 폐기한다.
장시간 MR 저하/어려운 quota/후처리 cache 및 실제 새 정책 비용은 별도 계측이 필요하다.
