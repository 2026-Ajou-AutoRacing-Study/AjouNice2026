# Base5 Sunny13시 직접 5K — 현재 운영 런북

2026-10-09 / v132 정지 자유 yaw·실측 parent 배치 형상·음영 접근/이탈 균형. v125 진단 metadata·v124 보충 유지.
Base5-only single-frame OD 수집이다. E25 추론·temporal 학습·rosbag 녹화 명령이 아니다.
**본5K는 사용자 승인 후 시작한다.** 아래 명령은 준비된 새manifest/output용이며 이번 검증에서 실행하지 않았다.

## 최신 준비 — v132

현재 release: `~/morai_cap/base5_v2_261008/final_appearance_5k_v132_release`.
본수집 manifest는 `environment_suite/sunny13/manifest.json`, 새 output은
`collection_5k_final_v132_sunny13`이다. 구 v123/v129/v130/v131 output을 새 코드로 resume하지 않는다.
5K/122종/정적4822→이동178/6환경30K, Park·정지·CollisionData·Z·visibility gate를 유지한다.
정지 비차량360° 연속 각도, OBJ 자유70/도로30·일반 정적 차량 도로70/자유30,
실제 이동/교통 NPC·횡단 맥락은 경로 yaw다. 형상 reference는 배치 계산용이며 native GT는 그대로다.
일반사람33셀gap0.6–0.8m 및반려동물동반3셀gap0.8–1.0m는실제spawn대조를근거로한제한prior다.
전수5K/6환경 및 최종 live·CPU/resume/비용/한계는
[v132 최종 근거](261009_base5_final_appearance_v132.md)를 따른다.
대표MR55/55·CPU55/107GT·완료resume추가0/raw1858불변·416tests PASS다.
가중시간외삽9.635h/5K, 운영예산10–13h·raw/QA가중32.572GiB/5K다. 후처리/복제공간은별도다.
8075는 새 root의 수집 전 상태로 전환한다. 아래 실행 명령은 v132만 사용한다.

### 이전 v129 준비 이력

사용자가 구Sunny13 수집을 중단했다. 기존763승인은 원본과 압축백업에 유지하며 강제resume하지 않는다.
현재는 새 `yaw_5k_v129_release` 입력과 **새 output**을 사용한다. 새본5K는 아직 재시작하지 않았다.
정지ped/AI·정적회피ped계열70%네방향±15°/30%360° yaw, 일반사람alongside33셀gap0.6–0.8m를 보완했다.
기존 거리/혼합budget/122종primary/정적4822→이동178/6환경30K는 유지한다.
전수5K·6환경 검증, 대표실제35/35·CPU35/91GT를 확인했다.
완료resume raw886불변/추가capture0·10session Park·403tests PASS다. 8075는새root의수집전상태다.
[최신 근거·대조·한계·백업](261008_base5_static_yaw_and_randomization_review.md)이 이하 구준비이력보다 우선한다.
master/bridge가 이미 살아 있으면 터미널3만 시작한다. 8075는 새output으로 전환한다.
캡처를 시작한 뒤 generator/helper/runtime/config/input을 변경하지 않는다.

### 이전 v125/v124 준비 이력 (당시 상태)

2026-10-08 대시보드 보강/본수집 준비: 사용자 확인은 **이번 Sunny13 5K, 이후6환경30K**다.
50K는 오타였으며 배치·거리·자산·GT·환경 계약을 변경하지 않았다.
v125 캡처 runtime은 유지하고 대시보드만 보강했다. 5000 OFFLINE_PASS/6환경 hash·group·split 확인,
외부 reference를 포함한 입력 백업, 브라우저 그래프 확인을 마쳤다.
[대시보드·최종 준비 근거](261008_base5_dashboard_and_launch_ready.md)를 따른다.

2026-10-08 13:05 KST v125: 기존 객체 trace에 원문 velocity/heading/source timestamp만 추가했다.
정지/달리기 새2셀 승인·CPU2/2GT·완료resume raw68파일 불변/추가capture0·375tests PASS다.
속도 학습/평가 GT가 아니며 추가 통신 설정·대기·촬영은 없다. 아래 본수집 명령/manifest는 그대로다.
`telemetry.json` schema3의 `object_motion_sample_contract`를 따른다.
runtime hash가 바뀌었으므로 이미 시작한 v124 output은 원래 revision `4d4fdb06`에서 재개하고,
새 코드로 강제 resume하지 않는다. 현재 본5K root는 미생성으로 확인했다.
[v125 기록 계약·검증](261008_base5_motion_metadata_v125.md)이 아래 v124 준비 기록보다 우선한다.

2026-10-08 12:14 KST 최종 확인: 현재runtime에서 새대표33/33·CPU/완료resume 및370tests 통과.
본수집 root는 아직 없다. [최종 EVIDENCE](261008_base5_prebulk_v124_final.md)를 따른다.
localhost8075 대시보드는 현재 root로 이미 실행해 `수집 전`을 확인했다. master/bridge/대시보드가
살아 있으면 중복 실행하지 않는다. 같은 설정을 유지하고 승인 후 터미널3만 시작하면 된다.
환경 manifest가 실제 MR UI 날씨/시간/센서를 자동 확인하는 것은 아니다.

## 이전 입력/6환경 계약 — v129 이력

Release root: `~/morai_cap/base5_v2_261008/yaw_5k_v129_release`.
Sunny13부터 시작하며 같은5K 배치를 sunny11/15, foggy11/13/15에 반복해30K를 만든다.
환경마다 UI에서 날씨/시간을 직접 적용·확인하고 다른 manifest/output을 사용한다.
manifest만 바꿔도 실제 날씨가 바뀌는 기능은 없다. 다른 환경을 같은 output에 resume하지 않는다.
`environment_suite/suite.json`에6manifest/output이 기록돼 있다.

당시 전수검사/구성/실제 검증 결과는 [v129 EVIDENCE](261008_base5_static_yaw_and_randomization_review.md)다.
고주로/로터리 전actor 차량만, 로터리350미션, 사람계열830범용·혼합장애물 보충,
독립음영500geometry유지·122종 primary 최소5회다. 정적4822부터 취득하고 이동178을 마지막에 둔다.
같은scene의6환경을 동일split으로 묶으며 독립geometry는5K이지30K가 아니다.
v129 대표live35/35·CPU35/91GT, 기존 완료 후 보충을 유지한다.
구output에 새runtime을 혼합하지 않고 본수집은 fresh output을 쓴다.
5K실측외삽9.326h/운영예산10–13h,6환경60–78h(후처리·학습별도)다. 첫100–200셀rate로갱신한다.
raw/QA외삽5K약26.81GiB/30K약160.86GiB이며캐시/압축복제/재시도공간은별도다.
위 수치는 해당 revision의 이력이다. 아래 본수집 명령은 v132 새 경로만 사용한다.

## 유지하는 실행 계약 / 과거 검증 범위

- 새5K offline 생성 완료: 정적4822/실제 이동178,122종 primary 최소4회.
- 음영500셀은 바깥먼접근60/가까운접근60/입구직전30/진입90/내부190/출구70이다.
  바깥145positive는 Ego밖/target안/전방/100m ROI를 동시에 만족한다. 오른쪽80/왼쪽20·좌우 엇갈림도 포함한다.
  음영122종 primary 최소1, 기존1750미션/초근접/범용 로터리와4500 entry/source 및음영24mover를 유지한다.
- 로터리 미션250셀은 샘플 NPC 시작→삭제 경로와 실제 Ego 통과 경로를 사용한다.
  접근20/대기30/진입60/원형 주행90/탈출50. 원형 주행 앞뒤 NPC/진입 두 대가 실제로 안 보이면 review다.
  범용 로터리100셀과 나머지4750셀의 배치 geometry/이동 높이 계약은 유지한다.
- 정지 차량도 D에서 움직이는 사례를 확인해 모든 정적 vehicleList actor의 저장 기어를 Park로 맞춘다.
  나머지4750 중2314셀은 위치/자산/GT 변경 없이 기어 상태·실제 위치 검사 metadata만 갱신했다.
  캡처 전후 기존 Object snapshot의 pivot XY가 계획 위치에서25cm 초과 벗어나면 review다.
  추가 촬영/높이 witness는 만들지 않는다. 원형 주행의 실제 GT 간격도 quota 범위±30cm로 검사한다.
- 이동 외관은 사거리154셀+음영24셀이다. 음영 외 범용/정적 미션은 정지 배치한다.
- 음영 이동은 사거리 lookup을 재사용하지 않고 independent terrain/static-reference/near witness로
  경로 높이를 준비한다. 도로 중앙/10–30m 관측·실제 이동/Z/무접촉 검사를 유지한다.
  부족한 경우에만 departure16m/12m 상보 witness를 추가한다. 확인되면 즉시 추가촬영을 멈춘다.
  기존 rigid 정적 자산은 움직이지 않고 차량은 반대/침범 방향을 포함한 Park 외관 snapshot이다.
- 수동 저장 높이를 초기 후보로 검증한 endpoint lookup의 domain 안에서 두 waypoint를 생성한다.
  중간 Z는 두 끝점의 선형 보간이며 자동 지면 추종이 아니다.
  미측정 구간은 같은 셀의 초기 생성 재시도이며 경로를 무제한 flat 외삽하지 않는다.
- all-editor mover만 terrain/static-reference 준비를 생략한다. 원본 capture는 계속 검사한다.
- 정적/검증된 이동 경로 모두 plane 부족·작은 높이 gap은 warning이다.
  신뢰 plane ±40cm 초과·심한 Z step·CollisionData 접촉·정지 실패/통신 이상은 제외/중단 정책을 유지한다.
- fresh transport의 all-static actor 누락은 원본 annotation으로 review하며 같은-quota 재배치/부족분으로 처리한다.
  이동 primary 또는 stale topic은 이 예외로 넘어가지 않는다.
- 원본 point/GT·실패 근거를 보존한다. 배치 수는 실제 visible-positive 수가 아니다.
- 이동 primary의 걷기/달리기·사람/희귀 그룹별 양방향 차이≤1, 재시도에서도 방향을 고정한다.
  [CURRENT](BASE5_V2_CURRENT.md)와 [근거/한계](261008_base5_intersection_ground_evidence.md)를 먼저 확인한다.
- 구 v113/v114/v115/v116/v117/v118/v119/v121 pilot output을 현재 코드로 resume하지 않는다.
  수집 시작 후 코드·config·manifest·scenario·lookup/source를 바꾸지 않는다.

v117 대표18셀18승인/21capture·184.685s는 이전 이동/혼합 검증 근거다.
v119 로터리 실제 캡처·43종·resume/CPU 결과는 [로터리 EVIDENCE](261008_base5_roundabout_route_mission_evidence.md)를 따른다.
최신 음영 release/실제 캡처·잔여 한계는 [음영 EVIDENCE](261008_base5_shadow_observation_evidence.md)를 따른다.
v122 새정적 대표20/20(실제23capture/161.432초), 바깥양성9/9 CAMERA_1·CPU20/29GT·완료resume
791raw불변/추가capture0은 [사전 접근 EVIDENCE](261008_base5_shadow_approach_evidence.md)를 따른다.
기존v121 정적25/25·이동6/7의 총31/32승인은 이력이다. Woman3 running1셀 지면 근거 부족은 남아 있으며
bounded 대체/부족분 정책을 유지한다. 전수동적합격/자동본수집승인으로 해석하지 않는다.
정적근접누락3review와일시적service응답오류2회는보존·복구했다. 세부사항은 EVIDENCE를 따른다.
새두점경로112개승인용검증은높이/운영scope이며 전체5K성공률 인증이 아니다.
전체소요는현재근거를조건부로합치면약10시간안팎이지만보장하지않고누적rate로갱신한다.
최종재확인은[CURRENT](BASE5_V2_CURRENT.md)의본수집전항목을따른다.
새5K/runtime계약·fresh3topic≈50Hz·무접촉/정지·약252GiB여유를확인했다.
추가높이개발은시작조건이아니다. 시작직전MR UI의Capture Mode/Sunny13/대회센서를유지확인한다.
음영 추가준비 포함 운영여유는10–13h이며첫100–200셀로실제ETA를갱신한다.

## 터미널 1 — master

```bash
ajou
rc
```

기존 master/bridge가 살아 있으면 중복 실행하지 않는다. 터미널들이 같은 ROS_MASTER_URI를 사용해야 한다.

## 터미널 2 — guarded bridge

```bash
ajou
python3 dataset/morai_od/scripts/run_base5_rosbridge.py
```

MR Capture Mode를 켜고 대회 센서 설정·Sunny13시를 UI에서 확인한다.
Ego Network Load→Connect:

```text
$HOME/AjouNice2026/configs/network/ego_network_mr_base5_savesensordata_collision_local.json
```

Ego/Object/CollisionData·ScenarioLoad·SaveSensorData·제어 서비스의7-interface profile이다.
**UDP receiver, sensing, localization, BEVFusion, planning, control을 별도로 실행하지 않는다.**
수집기가 Ego 정지/Park와 소량 임시 scenario 설치·회수를 맡는다.
5K 전체파일을 MR GUI 폴더로 복사하지 않는다. 같은 master에서 live 수집기를 동시에 두 개 돌리지 않는다.

## 터미널 3 — 승인 후 본수집 / 새 output

```bash
ajou
python3 -u dataset/morai_od/scripts/run_base5_batch_campaign.py \
  --manifest "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/environment_suite/sunny13/manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_final_v132_sunny13" \
  --limit 5000 \
  --chunk-size 4 \
  --approve-bulk-capture \
  --resume
```

- limit: 이번 campaign의 설계 셀 수. v132 환경manifest는 정적4822/이동178순서다. 5000으로 전체를 수집한다.
  실행 도중 limit를 바꾸면 같은output resume계약이 달라진다. 정적만 따로 하려면 시작 전 별도manifest/output을 준비한다.
- chunk-size: 공유 ROS child당 scene 수. 네트워크는 각scene reload하며 boundary/final Park를 확인한다.
- approve-bulk-capture: 64셀 초과를 승인한 실행용. 작은 smoke에서는 사용하지 않는다.
- resume: 기존 안전한 성공분을 검증 후 재사용. 첫 실행에도 가능하다. limit/chunk/계약을 바꿔 resume하지 않는다.
- 통신/정지/Park 불확실의 halted 상태는 자동 강제resume 대상이 아니다. 근거를 확인하고 안전 복구/새output을 결정한다.

## 터미널 4 — 대시보드

2026-10-08 수집 중 추가: 현재 캠페인 용량, 완료 시 예상 총용량, 앞으로 필요한 추가 용량,
완료 후 예상 디스크 여유를 표시한다. 승인 장당 현재 저장량(실패·재시도 포함)으로 외삽하며,
용량은 별도 백그라운드에서 5분마다 `du -sk`로 계측한다. hardlink는 한 번만 계산한다.
후처리·압축·다른 프로그램 저장량은 제외하며, 뒤쪽 동적 구간 비용 변화로 추정이 달라질 수 있다.
최초 용량표시 추가 때는 사용자 요청으로 테스트를 생략했다. 이번 v129 준비에서는 용량외삽/zero-approved/
hardlink단위검사까지 포함했다. dashboard만 변경할 때는 수집 runtime hash가 바뀌지 않는다.

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_final_v132_sunny13" \
  --manifest "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/environment_suite/sunny13/manifest.json" \
  --suite "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/environment_suite/suite.json" \
  --port 8075
```

브라우저 http://127.0.0.1:8075.
기존8081은구v45수집폴더를보는dashboard이므로이번5K진행률로해석하지않는다.
승인/부족분·누적 시간·승인 rate·디스크·현재 child/recovery를 확인한다.
보강 화면은 승인 누적/최근10분 승인속도 그래프, 패밀리별계획대비승인, 거리/방향 bin,
부족분 사유, 정적4822/이동178 검사 진행, 6환경30K 진행을 분리해 표시한다.
현재8075에 새 대시보드를 실행했다. 이미 실행 중이면 브라우저에서 새로고침만 한다.
차트는 대시보드 실행 이후 실제 관측으로만 그린다. 서버가 살아 있으면 탭을 닫아도 계측한다.
15초 표본을 `~/.cache/ajounice/base5/dashboard_<root-hash>.jsonl`에 남겨 서버 재시작 후 복원한다.
capture root에는 쓰지 않는다. `--once`는 journal도 쓰지 않는 일회성 확인이다.
최근 rate는 활성누적시간 최소60초/최근최대10분 기준이며 pause/오프라인 대기를 더하지 않는다.
큰raw/전체선택ledger를2초마다읽지 않는다. 작은status와quota checkpoint만 읽고manifest계획은시작때캐시한다.
부족분 사유는 최신quota checkpoint기준이고미반영사유가있을수있다. 여러사유가있는한장면은사유별중복집계한다.
정적/이동은전actor기준의원래검사순서진행이며이동승인수/속도GT인증이아니다. 보충중에는순서집계를쓰지않는다.
finished는 승인 수가 아니다. output 생성 전에는 not_started다.
waiting_stationary_ego는 정상 정지 검사 단계다. 준비 중 ETA0만으로 멈춤을 단정하지 않는다.

## 일시정지·재개

현재 chunk와 안전 종료를 마친 뒤 pause:

```bash
ajou
touch "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_final_v132_sunny13/PAUSE"
```

paused 확인 후 파일을 제거하고 터미널3의 **동일한 명령**으로 재개한다.

```bash
ajou
rm "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_final_v132_sunny13/PAUSE"
```

급히 중단하려면 부모터미널 Ctrl-C/STOP 경로를 사용한다. 종료 Park 확인 전 다른 수집기를 켜지 않는다.
부족분은 deficit_manifest/quota에 남는다. 끝난 뒤 아래 보충 명령으로 같은 설계 셀을 보충한다.
잘 나온 장면만 골라 전체quota 완료로 표시하지 않는다.

## 완료 후 부족분 보충 — v124

최초 수집이 `complete_with_deficit`로 끝난 후, MR의 환경/센서/통신을 그대로 유지하고 실행한다.
새 보충용 output을 만들거나 limit를 부족 장수로 줄이지 않는다. 원래 target5000과 chunk/manifest를 유지한다.

```bash
ajou
BASE5_ROOT="$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release"
python3 -u dataset/morai_od/scripts/run_base5_batch_campaign.py \
  --manifest "$BASE5_ROOT/environment_suite/sunny13/manifest.json" \
  --output "$BASE5_ROOT/collection_5k_final_v132_sunny13" \
  --limit 5000 --chunk-size 4 --approve-bulk-capture \
  --resume --replenish --replenish-rounds 2
```

- ordinary resume: 완료 근거 검증만 한다. 완료 후에도 부족분은 자동 재촬영하지 않는다.
- replenish: 기존 성공분/raw는 그대로, 부족 원래 셀만 새 geometry로 캡처한다. 동일 primary asset/spec/
  거리/방향/미션 단계/motion 요청/split/group/환경 config를 유지한다. 보조 배치 및 실제 raw는 달라질 수 있다.
- replenish-rounds: **셀당 전체 보충 회차의 누적 상한**이다. 기본2, 선택1–3이다.
  같은 명령을 여러 번 실행해도 상한을 초기화하지 않는다. 2회차 후 남았다면 명시적으로3으로 늘릴 수 있다.
  한 회차의 child에도 기존 최대2 quota round 및 bounded QA가 있으므로 실제 촬영2회를 뜻하지 않는다.
- PAUSE는 진행 중인 셀과 안전 종료를 마친 뒤 멈춘다. PAUSE를 제거하고 동일 보충 명령으로 재개한다.
  STOP/Ctrl-C가 진행 중 child를 끊으면 기존처럼 safety halt일 수 있다. 강제 resume하지 않는다.
- 완료 후 다시 ordinary resume 또는 동일 보충 명령을 실행해도 성공분/완료 child를 재촬영하지 않는다.
  상태 journal에 commit된 승인으로 ledger/quota publication을 복원한다. 원본 manifest는 덮어쓰지 않는다.
- 대시보드는 같은 output을 계속 본다. 최초 승인/추가 보충/잔여 부족/회차가 나오고, 보충 중 bar는 승인 수다.
  검사 finished5000이 학습 승인5000이라는 뜻은 아니다. `complete_with_deficit`는 부족분이 남은 종료다.
- `selected_ledger.jsonl`은 통합 결과다. 아래 coverage/CPU 변환 명령을 그대로 사용한다.
  `replenishment/round_XX/`에는 후보 manifest/child/raw/실패 근거, `state.json`에는 원래 셀 매핑을 남긴다.
- 환경별 보충은 해당 환경 manifest/output에서만 한다. UI 환경을 다시 확인한다. 보충 geometry가 바뀐
  환경은 완전히 동일 geometry의 weather A/B라고 주장하지 않지만, 원래 group/split은 유지해 leakage를 막는다.
- 미완료는 먼저 원래 resume으로 끝낸다. halted/코드·manifest·센서/hash 불일치는 자동 보충 대상이 아니다.
  이번 v124 도입 이전 output은 해당 revision에서만 검증/재개한다. 새 본수집에는 현재 코드를 쓴다.

[MR 통합 테스트와 보존/변환 근거](261008_base5_completed_replenishment_evidence.md).

## 최종 감사·CPU 변환

```bash
ajou
python3 dataset/morai_od/scripts/report_base5_final_coverage.py \
  --manifest "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/environment_suite/sunny13/manifest.json" \
  --ledger "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_final_v132_sunny13/selected_ledger.jsonl" \
  --output "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_final_v132_sunny13/effective_coverage.json" \
  --require-effective-complete
```

전체승인 장수와122종 실제 visible-primary 최소4회는 별개다. 최소4회는 학습 충분성 보장이 아니다.
GT는 raw Instance component 유지, LiDAR≥1점 또는 실제 camera Instance visibility로 positive를 정한다.
안 보이는 객체를 정답으로 넣지 않는다. LiDAR/Ego ROI는[-100,-30,-3,100,30,2]다.
accepted·review없음·calibration아님만 전달한다.

```bash
ajou
python3 dataset/morai_od/scripts/build_base5_bevfusion_infos.py \
  --ledger "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_final_v132_sunny13/selected_ledger.jsonl" \
  --output "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/infos_sunny13" \
  --sensor-config configs/sensors/Sensor_competition_local.json \
  --environment-tag sunny13
```

## 다른5환경25K

Sunny13을 마치면 MR UI를 다음 환경으로 바꾸고 Capture Mode/센서/네트워크를 유지한다.
아래 예시는 foggy11이며 sunny11/sunny15/foggy13/foggy15도 같은 방식이다.
동시에 여러collector를 켜지 않는다. 각manifest는 같은정적4822/이동178순서이며 split도 같다.

```bash
ajou
ENV_TAG=foggy11
BASE5_ROOT="$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release"
python3 -u dataset/morai_od/scripts/run_base5_batch_campaign.py \
  --manifest "$BASE5_ROOT/environment_suite/$ENV_TAG/manifest.json" \
  --output "$BASE5_ROOT/collection_5k_final_v132_$ENV_TAG" \
  --limit 5000 --chunk-size 4 --approve-bulk-capture --resume
```

변환도해당환경의 `--environment-tag "$ENV_TAG"`를 지정한다.6개infos를 모두 만든 뒤:

```bash
ajou
BASE5_ROOT="$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release"
python3 dataset/morai_od/scripts/merge_base5_environment_infos.py \
  --infos "$BASE5_ROOT/infos_sunny13" "$BASE5_ROOT/infos_sunny11" "$BASE5_ROOT/infos_sunny15" \
          "$BASE5_ROOT/infos_foggy11" "$BASE5_ROOT/infos_foggy13" "$BASE5_ROOT/infos_foggy15" \
  --output "$BASE5_ROOT/infos_30k"
```

신뢰하는로컬PKL에만사용한다. 같은group이서로다른split에있거나token이중복이면거부한다.
최종장수가부족하면부족분을명시한다. 변환/merge가자동으로30K완료를보장하지않는다.
원본보존+변환캐시+재시도로공간이증가하므로각환경시작전디스크를확인한다.

## 재현용생성 (본수집명령아님)

v132는 보존된 v129 입력과 승인 정지 raw에서 복원한 placement reference를 사용한다.
아래 output은 존재하지 않는 새 경로여야 한다. 기존 release를 덮어쓰지 않는다.

```bash
ajou
python3 dataset/morai_od/scripts/revise_base5_final_appearance.py \
  --source "$HOME/morai_cap/base5_v2_261008/yaw_5k_v129_release/scenarios_5k/manifest.json" \
  --reference "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/placement_geometry.json" \
  --output "$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_reproduction"
```

### 과거 v129 생성 이력

v129는 보존한v123에서 **존재하지 않는 새output**에 생성한다. 원본/현재release는 덮어쓰지 않는다.

```bash
ajou
python3 dataset/morai_od/scripts/revise_base5_static_yaw.py \
  --source "$HOME/morai_cap/base5_v2_261008/regions_5k_v123_release/scenarios_5k/manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261008/yaw_5k_v129_reproduction"
```

### 과거v123 생성 이력

v123은아래처럼검증된v122에서**존재하지않는새output**에생성한다.
음영500geometry와비로터리미션을보존하고범용/로터리350을재생성한다.
release전체manifest/동결spec/lookup/source를삭제하지않는다. 기본config만의generate로는재현되지않는다.

```bash
ajou
python3 dataset/morai_od/scripts/revise_base5_region_release.py \
  --source-manifest "$HOME/morai_cap/base5_v2_261008/shadow_approach_5k_v122_b/scenarios_5k/manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261008/regions_5k_v123_new"
```

release config 전체 내용과 runtime hash를 수집 시작 시 고정한다.
구v113 명령/4171+829/45.7h등은 Git 이력·CURRENT의 historical evidence에 보존하며 현재실행값이 아니다.
