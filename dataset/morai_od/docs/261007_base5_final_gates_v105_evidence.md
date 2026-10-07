# Base5 직접 5K 최종 gate — EVIDENCE

2026-10-07. Sunny11시 독립 Base5의 본수집 전 구현·실험 기록이다.
5K 본수집이나 학습·모델 정확도 검증 결과가 아니다. 구 10K 목표를 재개하지 않는다.

## 배치·GT 확인

- 차량 미션 근접133셀: 고주로66/로터리67. route-exact NPC, 경계 간격0.3–2.5m,
  앞옆·측방·측후방·앞뒤 밀집. live7/7, 실제 sensor annotation 간격0.508–2.002m.
  [상세 근거](261007_base5_vehicle_close_v100.md).
- 범용 근접175셀 및 정적 회피 alongside 정책은 유지한다. 원거리 quota를 줄이지 않았다.
- `gt_visibility_v100_live6`: 6셀 중5승인/1부족. 원거리 차량 셀139의 두 차량은
  LiDAR0점이나 실제 camera Instance 근거가 있어 유지했다. Nexo 중심은 약98m다.
  빈 배경은 GT0. BabyStroller2의5개 parent에서13개 component 중9개 positive를 유지했다.
  component 수를 객체 개수로 세지 않는다. CPU5장/18GT.
- `dense_v100_live3`: 셀685의7개 배치/7개 positive를 실제 확인했다.
  셀2764는8개 원안의 같은 quota 보완 후6개 배치/5개 positive다.
  8개 모두 실제 유지됐다고 주장하지 않는다.
- 셀3880은 보조 Man8이 Object 상태에 나타나지 않아 구 엄격한 로드 검사로 중단됐다.
  신선한 Ego/Object/Collision과 primary가 확인되고 빠진 것이 정적 secondary뿐인 경우는
  진단 캡처로 넘긴 뒤 `post_load_missing_static_secondary` review로 기록하도록 수정했다.
  누락 장면 자체를 성공으로 편입하지 않는다. missing primary/mover·통신 단절은 여전히 halt다.
  `population_recovery_v102`에서 같은 quota 재배치 후6개/6positive,4camera-only로 통과했다.

## 동적 혼합 지면·비용 문제

- 종전 near-terrain 준비는 이동 actor 최대3개, terrain 최대4세션에 묶여 있었다.
  실제5K에4개 이상 mover가 있는 셀이122개여서 범용 밀집 정책과 충돌했다.
  배치 최대8개에 맞춰 최대8target/9terrain으로 늘렸다. ground/Z/contact 검사는 완화하지 않았다.
- 별개로 움직이는 primary가 정상이어도 정적 보조 객체의 ground reference가 생성되지 않았다.
  모든 parent의 static target annotation footprint와 독립 empty terrain을 연결하고,
  높이가 실제로 떠 있으면 입력 Z만 보완한 뒤 새 실제 캡처에서 재검증한다.
  원본 annotation·센서 파일에 임의 지면 보정값을 덮어쓰지 않는다.
- 독립 plane·전체 footprint·파일 hash·0.1m 경로 height-step·실제 이동·최종 Z·CollisionData·
  Ego pose·Park gate는 유지한다. terrain이 모두 지원되면 추가 probe를 즉시 종료한다.
- `mixed_ground_v104`의 셀517은 이전 정적 보조 ground 실패에서 복구됐다.
  Woman3 + Car + Bicycle + SteelBarricade4개 모두 ground verified,
  gap −0.033~+0.008m,4positive/1camera-only다. 주 객체의 실제 움직임도 최종 검사한다.
- 여러 자연 이동 corridor가 한 장면에 겹치거나 지형을 벗어나는 사례는 별개 문제다.
  캡처 시점 외관을 위한 primary 이동 quota와 불필요한 secondary 동시 이동을 분리한다.
  최종 정책·새 live 결과는 아래 최종 동결 결과에 기록한다.

## 운영·감사

- static QA 예산을 최대8actor에 맞춰18단계로 조정했다. 무제한 retry는 아니다.
- 외부 same-quota replacement는 ground 실패 때 target 근처 지도 관측 가능 후보를 선별한다.
  departure 위치를 target 위치로 혼동하지 않는다. 사전검사는 접지 인증이 아니다.
- batch recovery에도 heartbeat를 연결했다. dashboard는 현재 recovery와 실제 capture stage,
  승인 수·누적 시간·시간당 승인·디스크 여유를 읽는다. localhost 전용, read-only다.
- converter는 accepted 표기가 있어도 review 이유가 남으면 제외한다.
- coverage 도구는122종의 배치 횟수와 accepted-visible primary를 별도로 집계한다.
  종류당4primary는 최소 coverage floor일 뿐 충분한 학습 인증이 아니다.
  `--require-effective-complete`가 있어야 실제 전체 수집 coverage를 최종 승인 검사한다.
- 비용 도구는 접지 calibration/실패/재시도도 포함하고 hardlink QA 파일은 한 번만 센다.
  일부러 고른 어려운 소량 표본의 외삽은 정상 수집 ETA 보장값으로 쓰지 않는다.

## 최종 동결 결과

- `final_5k_v105/manifest.json` 및 `final_5k_ordered_v105/manifest.json`은 각5,000unique/
  OFFLINE_PASS/오류0,train4,387/val613/test0이다.122종 planned-primary최소4장,
  regular/dense 예산·차량 근접133·범용 근접175·원거리 quota를 검사했다.
- 순서만 정적4,160→실제이동840으로 변경했고5000payload/hash/spec/split을 보존했다.
- 범용 동시 이동을 최대2actor로 제한했다. primary walk/run과 미션은 그대로이며
  초과 secondary만standing이다. 구성·전체 actor 수 예산을 줄이는 변경이 아니다.
  실제5K에서는7actor19셀/8actor30셀, 이전보다 resample평균1.366→1.2074였다.
  이것은 실제 수집시간 절감률이 아니다.
- `final_pilot_v105`:18셀 중15승인/3부족,820.485초 active wall.
  정적12/12, 실제이동3/6이다. 셀3은above-ground/reference-disagreement,
  셀110/274는target근처 독립 관측용 도로를 찾지 못했다. 성공으로 바꾸지 않았다.
  셀685와3036은각7개/7positive, mixed/static 재배치 및 camera-only/component gate를 확인했다.
- pause8→resume18에서 기존793sensor파일 hash/inode/mtime·선택8장 내용 불변.
  완료resume에서도2,919sensor파일·selectedledgerSHA·누적wall불변,추가촬영0이다.
- localhost HTTP dashboard의 `/` 및 `/status.json`, recovery→실제capture 상태 표시를 확인했다.
  일시정지/완료는 오래 갱신되지 않아도 통신 이상으로 표시하지 않는다.
- CPU:train13/val2,Vehicle11/Pedestrian19/Obstacle23/Animal4=57GT.
  raw annotation 보존,4unsupportedPedcomponent제외. calibration/부족분은편입하지않았다.
- 이 어려운18셀만으로 외삽한5K비용은정적24.78초/승인,동적174.36초/승인,
  가중약69.3시간/실패·QA포함약163GiB였다. **의도적 hard-case 표본이며 정상 ETA가 아니다.**
  그래도 과거18.7h/10K를 현재 예상으로 재사용하지 못한다는 경고 근거다.

### v106 후속 확인

혼합 재배치의 정적secondary preflight는12m를 허용했지만 동적 empty probe는8m였다.
동적혼합이면모든target8m,정적전용이면기존rigid-reference12m로같게맞췄다.
같은 목표·속도의6초/9초 출발여유를 대조했다. `budget6_live_v106`은5셀 중1승인/
4부족,436.873초다. 9초에서 통과했던62/517도6초에서는 unsupported였다.
환경·preflight 변경 영향도 있으므로 순수 시간만의 인과 대조로 해석하지 않는다.
6초가 낫다는 근거가 없으므로 기본9초를 유지한다. 실패 원안·원본을 보존했다.

### v107 지면 참조 정합

초기 empty terrain을6m뒤 도로 centerline에서 찍던 방식 대신 원래 Ego의XYZ/yaw/
횡 offset을 정확히 유지한 empty view를 사용한다.0m옵션이offset을 잃던 helper도수정했다.
독립 empty cloud에서는 ground review와 같은 Ego footprint를 제외한다.
raw는 바꾸지 않고10cm참조 불일치/전체 footprint/높이/contact gate도 그대로다.
Ego 메시지에 roll/pitch가 없으므로 yaw-only world 변환의 정확도를 과장하지 않는다.
이 수정만으로 모든 지형/동적 부유가 해결됐다는 뜻은 아니다.
`same_pose_ground_v107`은 이전 실패3셀 중1승인/2부족,299.303초다.
274는 같은quota재배치 후5actor 전체 verified,최저gap−0.039~+0.117m로복구됐다.
3/110은bounded재배치에서도static pivot terrain unsupported여서부족분으로남았다.
3의첫시도는참조불일치0.135m여서거부됐다. unsupported는실제부유확정과같지않다.
단지관측지면근거로승인할수없다는뜻이며gate를느슨하게바꿔성공처리하지않았다.

### 고정 seed 비용 표본

별도 `normal_cost_v107`에서 ordered v105의12셀을 확인한다.
seed261007105,정적10/실제이동2이며패밀리별표본이다. 결과를 본 뒤 교체하지 않는다.
indices:2007,3179,3072,1387,2091,2262,1690,3084,1253,1539,4779,4917.
위의 의도적인18hard-cell 외삽과 분리하고, 소량 표본의 조건부 ETA임을 명시한다.
실측12셀 중11승인/1부족,254.250초다. 정적10/10,실제이동1/2다.
3660은same-quota재배치에도terrain unsupported였다.4432는통과했다.
정적11.603초/승인·동적138.223초/승인,4160/840가중외삽45.660h다.
정적4160부분만13.4076h,unique QA/raw 포함단순외삽75.033GiB,표본allocated0.168GiB다.
동적2개뿐이고난이도/지형에민감해신뢰구간/속도보장은아니다.예전18.7h/10K는현재에사용하지않는다.
자유공간약257GiB이나최종bulk도디스크gate와실제누적rate를사용한다.

### v108 재감사 안전 근거 보존

원거리정적장면의raw를재사용하는ground-only audit가runner의capture_safety overlay를
생략해요약collision_verified가null로표시되는문제를발견했다. 원본metadata에는실제
CollisionData관측/no-contact·stationary support·collision_history 경로가남아있었다.
실제충돌검사를생략한사례로해석하지않는다. 다만선별도구는audit에서도그근거를필요로한다.
coreaudit에원본안전근거를deep copy하여보존하도록수정했다. 없는근거를성공으로만들지않는다.
실제검사·원본·motion/height gate는유지했다.
`reaudit_safety_v108`: 정적2403/3834와동적274/0을fresh실행해4/4통과,195.819초다.
선택4장의audit에실제no-contact근거가모두보존됐으며null요약을수정했다.
CPU4장/10GT(Vehicle2/Pedestrian5/Animal1/Obstacle2),부족분편입0이다.
완료resume후610sensor파일내용/inode/mtime 및선택ledger SHA·누적wall이그대로였고추가촬영0이다.
sensor digest:`51856c884c89d3867f6e7c0752e786451090949159dc743cbb008344583981e5`.
selected SHA:`d82836380ac14ed3144c61d4da15c95a3854dd0130b6e8f408aa58cb2b70452b`.
Base5단위검사238개PASS(19.893초),운용5000전수재감사OFFLINE_PASS/오류0다.
코드commit:`c802265b`(차량근접),`3bada1ec`(혼합/가시성/운영),
`3be3e651`(독립지면view/preflight/안전근거보존). Production/학습은미변경이다.

### 최종 운영 판단

배치/GT/범위/희귀자산/복합/혼합/근접/resume/dashboard 구현과대표검증은마무리했다.
모든동적지형이해결되거나122종전체실제coverage가충족됐다는뜻은아니다.
unsupported 동적 장면은부족분으로남기며정적4160장우선순서를준비했다.
전체5K를짧은밤수집이라고승인하지않는다. 본수집은사용자최종확인후다.
