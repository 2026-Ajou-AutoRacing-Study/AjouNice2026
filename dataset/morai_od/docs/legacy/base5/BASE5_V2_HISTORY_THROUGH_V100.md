# Base5 v2 시행착오·실측 누적 — HISTORY / v100 이전

## 환경 이력 정정 — 2026-10-07

이 문서의 Base5 기본 환경은 사용자 확인 Sunny 13시다. 최근 11시 표기는 오류였고
실측/ScenarioLoad 자동 적용이 아니었다. [정정 근거](261007_base5_sunny13_environment_correction.md).
과거 파일명의 `sunny11`과 원본 manifest 선언은 checksum/resume 때문에 보존한다.

2026-10-07에 CURRENT에서 분리한 원본 이력이다. 아래의 최신·다음·미완료는
각 작성 시점 표현이며 현재 실행 지시가 아니다. [현재 계약](BASE5_V2_CURRENT.md)과
[직접5K 런북](261007_base5_5k_capture_runbook.md)을 먼저 읽는다.

2026-10-07. 이 문서를 Base5 생성·캡처·변환의 최신 진입점으로 읽는다.
Base3 추가분이 아닌 독립 Base5이며 temporal/velocity head 학습용이 아니다.
**v45 본수집은 시작했다가 물리 충돌로 중단됐다. 체크포인트 32장은 보존했다.
새 코드의 소량 검증은 통과했지만 본수집은 자동 재시작하지 않았다. 본 학습도 미시작이다.**

## 최신 목표 변경 — Sunny13시 직접5K

사용자 요청으로이번목표를10K→5K로변경했다.
[최신구성·남은보완](261007_base5_direct_5k_current_plan.md)을우선한다.
생성기기본config는`base5_independent_5k_sunny11.json`이며구10K설정/산출물은보존한다.
아래10K및A/B각5K문구는변경전이력이다. 최종manifest는남은gate보완후새로만든다.
이번변경으로본수집을시작하지않았다.

## 다음 실행 — PLAN / 남은 보완

[v98 범용 근접 보강](261007_base5_generic_close_v98.md)은175셀 생성 가능,
좌우 대표2/2·CPU2장/GT6까지 확인했다. 원거리 quota는 유지하고 Ego를 도로 anchor에 둔다.
낮은 작은 객체의 초근접 가시성 실패는 해결하지 않았으며 성공으로 편입하지 않는다.
다음은 고주로·로터리 차량 근접 보강이다. 아직 최종5K 본수집 준비 완료가 아니다.

[최신 5K 계획의 다음 작업 순서](261007_base5_direct_5k_current_plan.md#다음-작업-순서--현재-실행-기준)를 따른다.
대상은 Sunny13시 직접5K이며, 구10K의 A/B 절반 선별은 더 이상 실행 계획이 아니다.
객체 수 정책은 구현·소량 검증 완료다. 남은 작업은 동적 장애물 미션의 위치 기준 수정,
범용/정적 회피 초근접, 고주로/로터리 밀집, 희귀 자산 유효 coverage 및 GT 대표 검증이다.
동적 미션의 기존 starter 기준 Ego −35~+55m와 Ego-relative 객체 배치는 v95에서
지도 횡단보도 기준으로 바꿨다. 접근/횡단 관측/양보90%, 통과 직후10%이며
250/250 조건 생성 가능성을 확인했다. 넓은 앞뒤 관측은 범용에서 담당한다.
차량 진입로 전체·모든 동적 자산 인증은 아니다.
[v95 검증·제한](261007_base5_dynamic_map_observation_v95.md)을 참고한다.
아래 v86 manifest는 이러한 보완 반영본이 아니다. 기본 고주로6m/로터리5m 중심거리만으로
초근접 충분성을 주장하지 않는다. 대표 검증 후 새5K manifest·전수감사·운영 명령을
고정하고 보고한 뒤 본수집 승인을 받는다.

### 1번 후속 v93: 범용 예산 / 미션 개수 구현·소량 검증

[v89 객체 수 보완](261007_base5_population_budget_v89.md)은구현했다.
현재5K의 범용positive3,000장에단일30/일반혼합50/밀집20%를배정하고footprint크기점수로
10–14/16–22점·최대8개를채운다. 미션은generic예산을사용하지않고family별
1–3개상황quota를유지한다. 현재512장오프라인검증까지통과했다.
배치actor와실제positiveROI parent·GTcomponent수를분리집계한다.
범용혼합에는최소2개·배치actor절반이상positive하한을추가했고calibration은제외한다.
다중QA→nearheight원본Park연결및tuple/JSON연쇄비교오류를수정했다.
최종v93 live4/4(70.76초),CPU4장/GT9,207tests PASS다.
6개장면의실제4개가시성은v91,7–8개는오프라인까지며전체조건인증은아니다.
이는범용/고주로/로터리초근접및최종5K coverage감사완료가아니다.
기존v86manifest에는이변경이없으며강제resume하지않는다.

## 최신 v86 — 본수집 전 통합검증 / 새10K 오프라인 준비

먼 BabyStroller1의 고유 rigid child를 anchor로 사용하되 모든 child GT 대응과
전체 footprint ground support를 검증하도록 확장했다. 원본 raw/GT는 미수정이다.
혼합Dog2는 near 실측 Z 재시도와 같은 quota의 ground 관찰 가능 후보 사전검사로
최종1/1(40.68초) 통과했다. 관찰 위치 사전검사는 실제 지면·가시성·충돌 인증이 아니다.
QA view의 campaign 검증 및 다중 actor 연쇄 QA 파일 분류 오류도 수정했다.

v82는32셀 중31승인/1부족분,486.30초였다. 8셀 pause 후 resume,
완료resume 추가촬영0/2,332파일 hash·시간 불변,CPU31장/GT44개를 확인했다.
남은 Ped_Pet1 동적 복합은 v84 near-empty terrain 보완으로1/1(37.51초) 통과했다.
v85 마지막8조건도8/8(164.32초),CPU8장/GT18개/sanity오류0이다.
이는 v82 단일 실행에서32/32였다는 뜻이 아니다.
동적 review의 내부 재배치는 금지하고 outer quota round에서 새 지형 준비를 요구한다.
최종 v86 동적2조건2/2(60.57초),단위회귀198 tests PASS다.

새 `/home/luv/morai_cap/base5_v2_261007/scenarios_10k_v85/manifest.json`은
현재 생성기/6초 natural-target/전체 actor ground 정책으로 생성했다.
10,000 unique payload,train8614/val1386/test0,전수OFFLINE_PASS/오류0이다.
검사기의 구 차량+0.36m 가정은 원본 F2·MGeo 후보 재계산 대조로 바로잡았다.
**전체10K live 인증·본캡처·본학습은 아니다.** 구 halted output 강제resume 금지.
최종 운용 후보는 `scenarios_10k_ordered_v86/manifest.json`이다.
payload/spec/primary/split을 유지하고 정적8,354→동적1,646 순서만 변경했다.
동적 준비가 필요한4셀 묶음에 정적 셀이 섞이는 overhead를 줄이려는 것이며
시간 단축률을 실측 없이 확정하지 않는다. [최신 운영 명령](261006_base5_bulk_resume_v45.md#2026-10-07-최신-v86--새-manifest--본수집-승인-대기).
이 최종 ordered manifest의 첫4셀 실제4/4(60.41초),전수OFFLINE_PASS/오류0도 확인했다.

강화된 QA를 포함한 v82의 승인31장 기준 단순외삽은 **약43.6시간/실제 시도 raw약103.6GiB**다.
coverage-biased 소량 표본이고 최종 정책 전체의 family-weighted ETA나 보장값이 아니다.
구18.7시간을 현재 예상으로 쓰지 않는다. 대량 진행 시 누적 속도·부족분을 다시 계측한다.
[실측 실패·보완·제한](261007_base5_prebulk_recovery_evidence.md).

## v77 — 보조 객체 원거리 검증·복합/동적 재확인

2026-10-07. 원거리 missing-ground 보완을 primary뿐 아니라 static secondary UID까지
확장했다. 단독 근접 calibration에서 동일 payload/실제 pose/rigid transform을 검증하며
원본 혼합 장면·GT는 그대로다. 복합 actor의 여러 component를 single-rigid로 인증하지 않는다.
구 readiness 도구의 natural-target→one-way-slow 덮어쓰기를 수정했고, 구형 이동 deck은
calibration 전에 거부한다.

초기 혼합4조건0통과→후속2통과/2review, 최종 secondary2/2(35.61초),
가까운 탑승자·유모차2/2(12.30초),
지형/정적 기준 준비를 포함한 실제 이동2/2(47.14초/6촬영)를 확인했다.
tuple/list QA 비교로 resume가 불필요한 보충 촬영을 하던 문제를 수정했다.
최종 완료resume1.104초/추가촬영0/hash불변/선택2장 유지, CPU 총6장/GT12개,
단위회귀183 tests PASS다.
먼 유모차 복합/혼합Dog2 ground 보완은 아직 미완료다. 본10K·학습은 미재개다.
[코드·실제 결과·비용·보류 조건](261007_base5_mixed_motion_ground_evidence.md).

## v72 — 원거리 정적 검증·원본 재검사 / 추가 촬영 감소

2026-10-07. 가까운 검증 plane과 같은 정적 rigid annotation의 전체 상대 R/T를 연결한다.
동일 actor payload/hash와 실제 XYZ/heading/size/정지 여부, Ego 수평 위치/yaw/tilt,
전체 footprint support를 검증한다. dynamic/복합/다른 placement에는 확대하지 않는다.
local plane의 실제 above/below 판정을 무시하지 않으며 yaw-only 장거리 Z 수정도 계속 금지다.

Kona 비교에서 yaw-only gap+17.33cm→full-relative−5.84cm로 차이23.17cm를 확인했다.
이는 변환에 의한 비교 오차의 근거이며 절대 Ego attitude/모든 부유 원인의 인증은 아니다.
실제 거리44/97/후방88/대형53m4조건 모두 통과했다. 같은4조건 전체시간78.62→53.60초,
원거리 원본 재검사로 재촬영4회를 없애 약31.8% 절감했다. 원본 metadata/audit와 sensor GT는
보존하고, hardlink raw와 별도 QA view를 남긴다. calibration은 학습 제외다.

최종1조건13.50초·완료resume0.104초/추가촬영0/witness hash불변, CPU 정상1장 변환 확인.
단위회귀179 tests PASS. 본10K 미재개/production 미변경. **18.7시간은 새 정책 확정 ETA가 아니다.**
미확인 높이/복합·먼 secondary 혼합/동적 전체경로/장시간 비용 분포 검증이 남아 있다.
[코드·검증 조건·실제 원본·비용·한계](261007_base5_static_rigid_ground_evidence.md).

## v66 — 정적 자산별 초기 높이 후보·전체 ground gate

2026-10-07. 정적 초기 배치에도 F2 저장 pivot와 원본 근처 MGeo 높이의 차이를 후보로
적용했다. 122종 중69종 후보/53종 fallback이며 **접지 인증69종이 아니다.** 차량 공통+0.36m는
후보 적용 시 중복 사용하지 않는다. 새 생성·준비는 모든 actor UID의 지면 검사를 요구하며
정적20m 밖 무검증 합격을 막았다. 누적 downward75cm/정적 차량 입력 retry/전체 BBox
footprint 평면 support/효과 없는 입력 수정 기록을 추가했다. raw GT는 미수정이다.

실제 `static_initializer_live_v65`는4조건 중3통과/원거리Kona1보류, CPU 성공3장/GT5개다.
Kona는 같은 실제 위치·heading의 근접 관찰에서는 ground gap−5.84cm로 통과했다.
따라서 원거리 gap을 실제 부유로 확정하지 않는다. yaw-only Ego world 변환의 cross-view
높이 오차 가능성 때문에 이 근거로20m 밖 입력Z를 낮추는 처리를 금지했다.
원거리 local ground가 부족하면 계속 review이며 근접 진단은 학습 quota에 넣지 않는다.

최종 Base5 회귀170 tests PASS. **구8354셀/v59 본수집 명령은 새 ground 정책 readiness가
아니며 자동 실행하지 않는다.** 본10K 미재개/production 미변경.
[코드·실제 결과·원거리 해석 수정·남은 한계](261007_base5_static_height_evidence.md).

## v59 — Ego Park 유지·확인된 충돌 실패분 분리

2026-10-07 후속 요청으로 기존 Drive 공통 전환을 제거했다. 정적·동적 캡처 모두 외부 제어
모드 + Park(gear1), 가속0/브레이크100%를 유지한다. 동적 객체 진행에 Ego Drive가 필요하다는
이전 설명은 철회한다. 실제 `park_dynamic_v49`에서 Dog1·2인 달리기·2인 걷기 3/3이
Park 상태로 이동·지면·Z·캡처 검사를 통과했다. 이전 v48과 다른 장시간 성공률을 주장하지 않는다.

확인된 CollisionData peer 접촉은 실패/부족분으로 기록하고 학습에서 제외한다.
장면 경계와 종료 Park가 검증되면 다음 수집 장면을 직접 로드한다. 별도 빈 복구 시나리오는
필수로 추가하지 않았다. 다음 장면의 새 위치·토픽·정지·접촉 검사는 그대로 유지한다.
전송 실패, 위치/토픽 확인 실패, 미확인 Park, 손상된 충돌 증거는 전체 중단이다.
접촉 없는 Ego drift를 추측으로 충돌 실패라고 간주하여 건너뛰지 않는다.

검증 근거는 `/home/luv/morai_cap/base5_v2_261007` 아래에 보존한다.

- `collision_input_live_v59`: 명시적으로 표시한 `INJECTED_TEST_CONTACT` 메시지 입력 시험.
  첫 장면은 SaveSensorData 미요청/충돌 실패로 기록했고 Park 확인 후 다음 정상 장면은
  실제 캡처·검증 통과했다. 정상1/deficit1, `complete_with_deficit`, 약11.73초.
  이는 **입력 실패 처리의 live 통합검증**이지 새 실제 물리 충돌 후 복구 인증이 아니다.
- 완료 resume는 ledger SHA·누적시간·센서파일 수 불변, 재촬영0이다.
  CPU 변환은 정상 장면1개/Vehicle GT1개만 포함하고 충돌 장면은 포함하지 않았다.
- 최종 코드 `park_final_v59`에서도 Dog1 실제 이동 약1.255m/s, Z step 약0.349mm,
  정상 캡처·종료 Park를 재확인했다. 기본 캡처 코드에는 Drive 요청이 없다.
  Base5 단위 회귀 158 tests PASS, py_compile 및 diff 공백 검사 PASS.
- 실제 물리 충돌 재현 시도 `collision_skip_live_v49`, `collision_npc_live_v50–52`,
  `collision_contact_live_v53–57`, `collision_cargo_live_v58`도 보존했다.
  접촉 신호 미재현/객체 로드 실패/정지 drift/진단 이동 guard 중단을 성공으로 계산하지 않는다.
  저속 Ego 이동은 당시 캘리브레이션 진단에서만 사용했고 **최종 수집 코드에는 해당 Drive
  예외나 이동 명령이 남아 있지 않다.** 실제 물리 접촉 직후의 복잡한 상태 복구는 추가 검증 영역이다.
- 이전 v45 실제 충돌 raw의 CollisionData 증거도 새 검증 함수로 검사한다. 과거 raw/세션을
  새 runtime hash로 강제 resume한다는 뜻이 아니다.

현재 자동 운용은 "확인된 접촉은 부족분, 나머지 안전/통신 이상은 halt"다.
실패분 자동 보충은 기존 quota 캠페인 방식으로 별도 수행한다. 정적 우선 본수집 후보는
동일 8,354셀/새 output `collection_static_first_v59`이며 아직 시작하지 않았다.
아래 v48 내용은 이전 구현·검증 이력이며 충돌 시 전체 중단 정책은 위 정책으로 대체됐다.

## v48 이력 — 실제 충돌 원인 수정·지형 기반 동적 경로

- 중단 근거: `collection_10k_v45`의 `b5v2_00033`, Bicycle_city(UID8500).
  미측정 자산의 1×1m proxy/offset 0으로 계획한 약 0.294m 간격이 실제 annotation에서는
  Ego와 약 0.0846m² 겹쳤고 CollisionData에도 해당 UID가 기록됐다. 동적 높이 문제만으로
  설명되는 중단이 아니다. 단순 통신 실패로 간주하거나 자동 resume하지 않는다.
- 미측정 Obstacle 배치 proxy는 3×3×2.5m로 보수화했다. 초근접 회피 간격은 측정 자산
  최소 0.30m, 미측정 자산 최소 1.0m다. 기존 quota의 요청 간격도 별도 보존하며 실제로
  10–30cm를 확보했다고 주장하지 않는다. proxy는 배치용이지 학습 GT가 아니다.
  동일 Bicycle 보완 장면은 실제 annotation 간격 1.282m/겹침 0/CollisionData 접촉 0으로 통과했다.
- 동적 경로는 빈 지형 LiDAR와 같은 자산의 정적 기준 캡처로 준비한다. 경로를 0.5m 이하
  간격으로 지형에 맞추고 자산 pivot offset을 적용한다. 지형 외삽, 근거 없는 일정 Z,
  큰 지형 단차는 로드 전에 거부한다. 원래 목표 XY·속도·준비 시간은 유지하되 측정 가능한
  안전 경로가 필요하면 이동 방향은 변경하며 그 변경을 기록한다. Ego와 다른 객체의 전체
  이동 corridor를 함께 검사한다. 정적 Obstacle/rigid 자산은 움직이지 않는다.
- 캡처 후에도 모든 이동 UID의 지면·Z jump·실제 이동 여부를 검사한다. 20m 밖이라는
  이유로 이동 객체 지면 검사를 생략하지 않는다. 통과는 annotation 기반 지면 proxy와
  수신 샘플 창의 검증이지 모든 mesh 접지/전체 주행의 인증이 아니다.
- 새 ScenarioLoad 직후 이전 장면 CollisionData가 한 번 남는 사례를 확인했다. 새 Ego
  위치·yaw와 일치하는 최초 메시지 이전의 전환 prefix만 제외한다. 그 이후의 충돌·충돌로
  인한 Ego 위치 변화는 모두 유지한다. 실제 접촉 확인 시 SaveSensorData 요청 전에 중단한다.
- 캘리브레이션 캡처는 학습 quota/infos에 넣지 않는다. 원본 Instance GT는 수정하지 않으며
  ROI `[-100,-30,-3,100,30,2]`, `lidar1_camera0`, animal 별도 class 정책은 유지한다.

### 실제 검증과 한계

근거 루트: `/home/luv/morai_cap/base5_v2_261007` (Git 비추적).

| 검증 | 결과 |
| --- | --- |
| `static_capture_v46b` | 정적 7/7, 36.93초, 종료 Park 확인 |
| `static_first_capture32_v46` | 정적 32/32, 지면 보완 1회 포함 33촬영, 174.76초, 접촉 0 |
| `static_first_infos32_v46` | train29/val3, GT42개, 원본 보존 |
| 초기 동적 v46 | 1/4; 지형 근거 없는 3개는 거부. 무작위 보충만으로도 해결되지 않음 |
| `motion_direction_capture_v47` | Woman2/Deer1/Ped_Pet1 3/4; Dog1은 지형 근거 부족으로 보류 |
| `motion_mixed_capture_v48` | Dog1 + 2인 혼합 2장면, 3/3, 69.47초, 접촉 0 |
| v48 이동 상태 | Dog1 약1.207m/s, 달리기 2인 약2.183m/s, 걷기 2인 약1.109m/s |
| v48 수신 창 최대 Z step | 5개 이동 객체 모두 0.000382m 이하, 지면 gate 통과 |
| CPU 동적 변환 | v47 3장/GT4, v48 3장/GT5; 캘리브레이션 제외 |
| `final_resume_v48` | 최종 코드 4셀 경계 정지→8/8 재개, 누적41.625초, 완료 resume ledger SHA·누적시간 불변 |
| `final_resume_infos_v48` | train7/val1, GT12개, review/실패 편입0 |
| 단위 회귀 | Base5 155 tests PASS |

Woman2 경로는 지형에 따라 Z가 약0.318m 변하지만 캡처 시 지면 gate를 통과했다.
즉 경로 Z 변화 자체가 뜸 버그는 아니며 지형 대비 높이와 불연속이 기준이다.
Dog1은 빈 지형 관측 Ego를 12m 대신 6m 뒤에 놓아 지면 근거를 확보했다.
현재 여러 이동 객체의 안전 corridor가 확보되지 않으면 해당 quota는 보류한다.
위 결과는 모든 자산/10K 장시간 성공률을 보증하지 않는다.

### 다음 본수집 후보: 정적 8,354셀 우선

`static_first_all_v46/manifest.json`: 원래 요청 family·primary asset·split을 유지하여
현재 안전 배치 정책으로 정적 8,354셀을 재생성했다. 전수 OFFLINE_PASS/오류0이다.
동적 1,646셀은 별도 지형 준비가 필요해 보류했으며 정적으로 바꾸거나 quota에서 지우지 않았다.
오프라인 PASS는 live 성공 보장이 아니다. v45의 18.71시간 예상도 새 동적 캘리브레이션에
그대로 사용할 수 없다. 구 raw 약184MiB는 중단 근거로 보존한다.
최신 실행·재개 주의는 [운영 런북의 10월 7일 변경](261006_base5_bulk_resume_v45.md#2026년-10월-7일-충돌-중단-후-운영-변경)을 따른다.
아래 v45 이하 내용은 당시 검증 기록이며 현재 본수집 상태가 아니다.

## 최신 v45 — 6초10K 준비·bulk checkpoint·안전 PAUSE/resume

[실행·정지·재개 런북 및 근거](261006_base5_bulk_resume_v45.md).
새`scenarios_10k_budget6_v45/manifest.json`은10K/unique10K/OFFLINE_PASS,
train8607/val1393/test0, 기존quota/Ego/목표geometry/자연속도/split 유지.
본수집 모드에서는4셀 child 유지, 전체목록64셀 checkpoint와 lightweight dashboard.
명시적bulk승인 없이64셀 초과거부, halt 자동resume거부, 누적active wall 보존.
실제12/12(14촬영/108.464초)과PAUSE marker8/8(8촬영/51.624초) 통과.
최종8셀완료resume ledger SHA/시간/호출수불변·추가촬영0. CPU합20장/GT28,143tests PASS.
9초v35 baseline은보존, 다음본수집은 별도6초v45 deck을선택한다.
예상18.71시간/재시도가정19–20시간대 유지. 새I/O의추가절감·하루안정성은미측정.
**본10K 촬영·학습 미시작.**

## v44 — 후방·복합 및 새 자산64셀·시간 민감도

[실제 결과·10K 시간 계산](261006_base5_diverse_coverage_eta_v44.md).
후방/복합8/8, 새자산혼합56/56 통과, 합66촬영/64장/GT95.
각68.263/314.356초, Event합152회·짧은오류2회retry회복·timeout/미종료worker0.
canonical10K에서 자연 이동1,646셀:9→6초1회촬영 절약은약1.37시간.
대표 혼합표본10K 예상18.71시간 유지. 자연 이동5–10%재시도/건당추가30초를
가정하면19.40–20.09시간이며 실제실패율이아니다. 정적56셀외삽15.61시간은채택하지않는다.
138tests PASS. 기본9초/production 유지. 본10K 캡처·학습 미시작.

## v43 — 6초 준비 확대·목표 누락 복구 실측

[실측·안전 fallback 근거](261006_base5_budget6_fallback_v43.md).
14종 자연 이동16/16 통과(19촬영), 혼합32/32 통과(36촬영/216.00초).
동일 방식10K 예상20.19→18.71시간: 소량 표본 외삽이지 완료 보장이 아니다.
준비5초 지연 진단은6초 경로에서 캡처 없이 안전 실패→동일quota9초 재시도 통과.
진단 데이터는 calibration_only이며 CPU 변환 train/val/test0/0/0으로 제외됐다.
재시도는 초기 목표 도착 실패 중 안전 증거/Park/SHA 확인된 경우만 허용한다.
canonical10K 기본9초 유지. 본10K/학습 미시작.

## v42 — 단일32셀 검증·비용 재집계·동적 준비 후보

[비용·pause/resume·snapshot 수정 근거](261006_base5_batch_capacity_budget_v42.md).
한새campaign32/32 통과, 실제36촬영/233.02초(v36 314.657초대비약26% 단축).
EventCmd86회·짧은전환오류3회retry회복·timeout/미종료worker0, CPU32장/GT48개 PASS.
같은방법의family가중 외삽27.24→20.19시간이며작은coverage표본의예상일뿐이다.
별도chunk8 pause/resume16/16·호출37회무오류 PASS. 기본chunk4 유지.
동적4셀에서같은속도/목표위치, 출발여유9→6초후보59.39→44.58초(약25%).
양쪽4quota/5촬영/최종gate PASS,6초후보CPU4장8GT PASS. canonical10K 기본9초는미변경.
완료resume와변환기동시읽기경합을고쳐부분ledger발행을막고원래완료wall을보존한다.
132tests PASS.최종새2셀완료resume는25회동시읽기/ledger SHA/wall불변·추가호출0.
32장학습형식은`batch_32_seed_guard_v42_infos_final`만사용한다.
다음은동적자산/지연조건확대·6초실패시동일quota9초fallback 검증이다.
**본10K 캡처·학습 미시작.**

## v41 — guard live 확인·중복 촬영 제거

[실측·안전·resume 근거](261006_base5_guard_seed_recovery_v41.md).
사용자 재연결 후 Park12회/남은4셀 보완4/4 통과로 기존선택과 합쳐 대표32/32다.
같은4셀에서 review 재사용으로 실제촬영6→5회, wall53.93→41.95초(22.21% 단축).
최종보완Z는 각실측값으로 산정하므로 최종payload 비트일치 비교는 아니다.
timeout/미종료worker0. 새실행에는 reload전환 오류2회가 약0.21초에 반환됐고
기존retry 후 정상응답·모든Park확인. 무오류/장시간 인증/전체10K22% 절감을 주장하지 않는다.
seed는 원본session·payload·audit·Park를 검증하며 gate나network reload를 생략하지 않는다.
123tests PASS, 완료resume 추가촬영0, CPU4장/GT7개 변환 PASS.
대시보드8077은 새seeded campaign이다. 다음은 더넓은 bounded throughput/stop-resume 검증.
**본10K 캡처·학습 미시작.**

## v40 — rosbridge 서비스 세대 응답 혼선 재현 / 당시 재연결 필요

[원인 근거·보완 코드·재연결 순서](261006_base5_eventcmd_rosbridge_v40.md).
정지Park12회는모두정상응답했고,캡처timeout은TCP요청후응답수신대기로확인했다.
설치된rosbridge에서서비스재등록시ID재사용/공유response dict로이전handler가새응답을
가져가는결함을offline재현했다.세대분리guard는재현테스트를통과했으나MR live미검증이다.
client timeout연결정리·미종료worker후추가요청금지·phase/stack기록을구현했다.
추가8quota통과로최종28/32셀.남은4셀중3raw는최종Park미확인으로선택하지않았고1셀은미로드다.
925셀동물annotation지면위review도보존하며안전복구후기존Z보완루프로검증해야한다.
기존bridge터미널종료→`python3 dataset/morai_od/scripts/run_base5_rosbridge.py`→MR재연결이필요하다.
설치ROS파일/production alias는미변경.모든timeout원인해결이나전체32PASS를주장하지않는다.
Base5단위118tests PASS. guarded bridge/MORAI ID echo호환성은재연결후검증한다.
**본10K 캡처·학습은 미시작이다.**

## v39 — 안전 ROS batch·quota 보완·resume 검증

[재연결·실제 검증·새 pilot 실행](261006_base5_safe_batch_quota_v39.md).
재연결 사전Park 통과 후 첫 batch의 boundary Park2회timeout으로 중단했다.
최종Park는 확인했고 실패 세션은 보존한다. 별도 새6장 pause/resume는6/6통과했다.
chunk4의 첫12셀은 보완 후12/12통과,14회 실제촬영/최종quota12장이다.
완료resume 원본aggregate SHA 불변,CPU12장/17GT 변환 통과.
남은20셀은8quota통과 후 다음chunk 최종Park2회timeout으로중단했다.
합계최종20/32quota·29회촬영(terrain포함), 미확인chunk4raw는선택에서제외했다.
신규driver포함109tests PASS. 실패캠페인resume금지·남은12셀은안전복구후새output검증이필요하다.
후속parent Ctrl-C/child process-group 정리 보강은단위검증이며추가live는미실행이다.
driver hash가바뀌므로구v39도현driver resume금지다. live코드는84ff5887로보존했다.
사후별도Park17.9ms정상응답·선택20장/GT28개CPU변환·후속112tests PASS.
사후Park성공은실패chunk승인이나장시간안정성해결이아니다.
네트워크는 매scene reload하며, 통신/충돌/Park 실패를 quality retry로 덮지 않는다.
새 batch driver는최대64셀 pilot이며 본10K수집용기본이 아니다. 간헐Park 원인은 미해결이다.
**본10K 캡처·학습은 미시작이다.**

## v38 — ROS 공유13.85% 단축 / 네트워크 재사용 실패

[실측 비교·실패·복구 순서](261006_base5_ros_batch_network_evidence_v38.md).
동일6개 셀별프로세스50.484초/ROS공유43.491초, 양쪽6/6통과와 최종Park 확인.
네트워크첫장면만 reload하는 실험은 두 번째에서 클라이언트0·위치timeout·종료Park미확인으로 중단했다.
세 번째는 미로드. 해당 옵션은 코드에서 금지했고 매 장면 network reload를 유지한다.
max16 shared-ROS 안전batch 후보/103tests를 추가했으나 새 guard+stop/resume live는 재연결 이후다.
당시 통신 복구 전 추가로드를 중단했다. 복구 이후는v39를 따른다. 본10K/학습 미시작.

## v37 — 비용 계측·중복작업 제거 / live6셀 A/B

[단계별 비용·변경·정확일치 근거](261006_base5_capture_stage_optimization_v37.md).
동일6개 시나리오56.556→50.732초(10.30% 절감), 양쪽6/6통과·종료Park 확인.
고정 preflight1.3초를 실제 연결/0.3초 정지 증거 기반으로만 단축했다.
원본32장 cache on/off 감사출력 정확일치,10K manifest 전체 반복복사 제거, tests100PASS.
로드/제어/파일 안정화·충돌/지면/Z/가시성 gate는 유지한다.
구세션은 코드hash 불일치로 현코드 resume 금지, 원본보존 및 읽기전용 보고서만 허용한다.
본10K 캡처/학습은 아직 미시작이다. 하루 규모 비용이라는 판단은 유지한다.

## v36 — 대표32셀 live 통과 / 본 수집 비용 확인

[실측 시간·용량·보완 근거](261006_base5_capture_capacity_v36.md).
32/32셀 통과, 실제촬영36회, 총314.657초. 보완3셀과 모든 종료Park 확인.
현재 방식의10K family 가중 예상27.53시간, 원본 저장량 단순 예상54.71GiB다.
작은 coverage 우선 표본이며 장시간 안정성/완료시간 보장은 아니다.
본10K 캡처·학습은 아직 미시작이며 canonical v35는 유지한다.

## v35 — 10K 시나리오 전수 PASS / 실제 수집 대기

[경로·분포·실패 수정·남은 제한](261006_base5_final_10k_v35.md).
canonical은 `$HOME/morai_cap/base5_v2_261006/scenarios_10k_v35/manifest.json`이다.
10,000 unique payload/errors0, eligible122종 primary coverage, Base5 tests94개PASS.
v34의1건 moving secondary geometric-center 경계 초과를 generator guard로 수정하고 재생성했다.
로터리 actual 우측2장 편향과 미검증 geometry/20m밖ground 한계를 숨기지 않는다.
MR 시나리오 폴더에10K복사/실제캡처/학습은 하지 않았다. 구10K·v34는 수집에 사용하지 않는다.

## 공식 답변 반영 — 2026-10-06

후속 **v33 코드 반영 / 12-cell live 완료**:
[공식 정책·12장 준비 및 검증](261006_base5_official_reply_smoke.md).
열린 G80 공식 제외와 rigid/차량 정지 캡처를 코드에 반영했다.
12장 offline PASS 뒤 사용자 재연결로 실제12/12셀 통과·부족0이다.
첫 셀 pause/resume·완료 read-only resume, CollisionData/종료Park, CPU12장/GT30개 변환을 확인했다.
동적5셀 capture-window Z step0m. 앞96.86m LiDAR0/뒤95.83m 4점은 camera 근거로 보존했다.
20m 밖 ground 인증은 아니며 재촬영은 이번에는 필요 없었다. 본10K는 미시작이다.

[MORAI 추가 답변 기록](260920_questions_for_morai.md#최신-추가-답변--객체-범위g80차량-속도-기준):
F2 전체 에셋이 음영/정적/동적 장애물 구간의 출제 후보이며 기존 정적 객체를
움직이는 예외는 없다. `OBJ_G80_Open_Door`는 공식 출제 제외/목록 삭제 예고다.
Animal/복합 객체 coverage는 유지하고, rigid static 자산에 양수 goal velocity를
넣는 스트레스 실험을 대회 분포로 확대하지 않는다. 부유 재현 ZIP은 공유 Drive 업로드 및 사용자 전달 완료다.
Z/ground gate를 유지하며 G80 제외가 타 자산 부유 해결을 의미하지 않는다.
전체 후보를 모든 자산 균등 sampling/동일 Obstacle GT로 해석하지 않는다.
나무/불/도로시설 background/ignore 정책은 공식 출제 금지가 아니므로 coverage 재검토가 남는다.
UDP 차량 velocity의 후륜 기준도 공식 확인됐다. OD 학습은 센서 geometric box
그대로이며 속도 평가 계약과 구분한다. 최초 문서 갱신에서는 코드 미변경이었고,
후속 v33에서 생성기 정책만 반영했다. 10K 생성/평가기 기본값은 변경하지 않았다.

## 최신 v32 — MR OFF batch1 / ROS 포화 비용 확인

5060Ti에서 E25 batch1 service71.94ms/13.90FPS, Base5320×896/±100m freshgeometry 후보94.52ms/10.58FPS.
E25 ROS원속도 입력/출력11.47Hz·드롭0, 약98Hz입력 stress에서는출력12.97Hz/평균receipt→response88.89ms다.
학습되지않은Base5후보의정확도·4090·확대후보ROS·full-stack 성능은측정하지않았다.
4-class/resize 온라인연결은후속이다.10K 본수집은미시작이다.
[측정계약·원본·재현](../../../docs/bevfusion/261006_BASE5_BATCH1_CAPACITY_EVIDENCE.md).
[단계별 원인](../../../docs/bevfusion/261006_BASE5_LATENCY_STAGE_BREAKDOWN.md):
ROI확대비용을0.10mvoxel/0.40mBEV/1mdepth가상쇄했다.기존격자+0.5mdepth의100m highres대조는모델처리약1.98배다.
현재격자의원거리정확도유지여부는미확정이다.
최신간격고정3-class/200query 대조는기존66.48/이미지만90.46/범위만96.27/
둘다130.59ms다.0.075mvoxel/0.30mBEV/0.50mdepth 간격을고정했으며
범위확대depth bin수는118→198이다. production격자변경또는확대모델정확도평가는아니다.

## v31 — 남은 조건 통과 / bounded campaign quota·resume 연결

남은 Deer1+바리케이드, 유모차2개, 스쿠터+Niro 혼합3셀을 재검증해 모두 통과했다.
Deer는 독립 지면 보완, 스쿠터 혼합은 누락 이후 같은 셀 재배치가 필요했다.
`run_base5_capture_campaign.py`로 quality loop를 대량 수집 경로에 연결했다.
원래 셀당 한 장만 quota로 인정하며 유한 재시도 후 부족 manifest를 내보낸다.
통과 셀은 resume에서 raw/session 근거 검사 후 재사용하고, 안전 종료 미확인 시 재개를 막는다.
실제 소량 stop/resume 검증 및89 tests PASS.10K/장시간 인증·학습·GPU속도는 미실행이다.
[v31 EVIDENCE](261006_base5_campaign_recovery_evidence.md),
[실행·resume·보충 명령](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md#v31--quality-campaign-resumequota-보충).

## v30 — 입력 Z 여유 가설 시험 / 실측 지면 보정 유효

Dog2/Ped_Pet1/Scooter1 정지·이동에서 기존 입력+0/2/5/10cm, 지면 보정 후 같은 여유,
보정 후−5/−10cm를 실제 MR에서60회 시도해57 raw를 보존했다. 도착미확인3회는 저장 차단.
작은 상향 여유는 부유를 해결하지 않고 대부분 그만큼 더 높였다.
Scooter1 정지는 지면에 내려앉지만 이동은 runtime Z가 입력보다약34cm 높게 유지됐다.
안전 격자 단위로 튀는 가설의 근거는 얻지 못했다. 장면 확인 이후 큰 Z step도 미관측이다.

실측 지면 기준으로 입력/waypoint Z를 함께 보정한 대조군은24/24조건 전체 gate 통과.
대표 annotation-ground gap은 Dog2정지+28.6→−2.9cm, Ped_Pet1이동+21.6→−2.1cm,
Scooter1이동+59.7→−2.9cm다. 이 위치/모드의 측정량을 전역상수로 적용하지 않는다.
**일괄 Z 상향은 채택하지 않고 v29의 관측→입력보정→재촬영을 유지한다.**
모든raw에서Ego-peer 비충돌,3세션 종료Park 확인. 간헐첫timeout/재시도 성공은 남아 있다.
아래쪽 대조군은 학습 선택 금지. Base5 tests82개 통과.
다음은 남은 근접·혼합3cell의새위치 검증, 이후MR 종료 후GPU 속도다. 본10K/학습은 미시작.
[v30 원본·수치·제한](261006_base5_z_margin_probe_evidence.md).

## v29 — 무효 LiDAR 지면 오판 수정·자동 보완 구현 / 당시 Park 실패로 중단

근접 below-ground 의심의 일부는 **검사기 오판**이었다. 원본 Dog2 근접 cloud의
zero XYZ 점10,010개가 extrinsic 후 같은 높이로 모여 평면 적합을 왜곡했다.
지면/visibility 감사에서 zero XYZ와 Ego footprint를 제외했다. raw cloud/GT 및 모델
추론 전처리는 바꾸지 않았다. 기존17장 재검사는13 accepted/4 review이며,
근접·혼합13장 중9장 통과/4 review다. 아래 v28의 낮은 합격률은 당시 이력이다.
과거 개 표본의 실제공중gap25–29cm 및 입력Z 수정 효과는 별개로 유지된다.

근접 ground 부족에는 empty-scene/경로상12m 뒤 Ego의 독립 지면 촬영과 world-plane
reference를 연결했다. 로컬·독립 평면이10cm 이상 다르면 통과시키지 않는다.
`run_base5_quality_loop.py`는 최대4cell에서 캡처→입력Z 수정/독립지면→같은cell 재배치→
재검증을 자동 수행한다. 각cell 최대2retry, 안전 실패 시 후속로드 차단, STOP/status/partial
selection 보존이다. 일반 capture runner의resume은 유지하되 loop campaign은 새output을 쓴다.

새12raw(지면5+객체7) 중 안전 종료된 객체4장만 변환했다. train4/GT8개,
RGB/LiDAR/캘리브레이션/4-class/7D/±100m×±30m LiDAR-frame ROI CPU 검사PASS,
raw annotation 대비 GT차이0. 학습/모델 성능 검증이 아니다.
독립 Woman2/Dog2/Ped_Pet1 재촬영3조건은 높이·Z·가시성·충돌·종료Park 통과.
자동loop 첫cell도 보완 성공. 두 번째cell은 raw검사 통과했지만 **종료Park2회timeout**으로
즉시중단했다. 해당raw는학습선택에서제외했고나머지2cell은로드하지않았다.

Base5 tests76개통과. 다음은 사용자가MR 재연결/서비스 정상 확인한 뒤 중단/미실행조건
소량마무리다. 실제 지형의 최초배치높이를항상맞추는universal solver는미완성이며
20m밖/이동경로전체ground contact인증도아니다. 10K/학습/GPU latency는미실행.
[v29 근거·원본 경로](261006_base5_ground_quality_loop_evidence.md).

## v28 — 일반 생성기 통합 이력 (지면 검사 오판 후속은 v29 우선)

자산별 외관 정책과 자연 속도 뒤쪽 출발을 일반 `generate_base5_independent.py` 기본에
통합했다. 확인된 fixed-pose 자산은 정지 우선, 사람·동물은 요청된 동적 phase 유지,
미검증 자산은 요청 보존이다. `natural-target-v1`은 목표 거리 quota와 전체 경로 안전 검사를
분리하고 실제 목표 도착 후 캡처한다. 전체 10K는 새로 생성하지 않았다.

근접·혼합 9장+재배치 4장을 촬영했다. 캡처 window의 동적 Z 급변은 없었지만
최종 엄격한 로컬 지면 검사에서는 13장 중 2장만 전체 통과했다.
나머지는 지면 미확인·지면 아래 배치 의심·누락으로 review이며 raw를 보존했다.
**안정된 Z ≠ 지면 정상**이다. v27의 parent Z matched 비교를 지면 보정으로 읽지 않는다.
기존 개·사슴·반려동물 동반 사람 일부는 주변 지면보다 약27–29cm 떠 있었다.

입력 scenario/waypoint Z만 낮춘 별도 재촬영 도구를 구현했다. Dog1/2 정지·이동
4조건에서 +25–29cm의 GT 하단–지면 gap을 약−3cm로 줄였고, 실제 이동/Z/가시성/
CollisionData 검사도 통과했다. raw GT 사후 보정은 없다. 지면 plane 미확인 장면은
억지 보정하지 않는다. 20m 밖·모든 이동 경로의 ground support는 인증하지 않는다.

Base5 unit tests70개 통과. 모든 세션 종료Park 확인, 간헐 EventCmd timeout은 재시도 성공.
최종 코드 재검사 정규 raw17장 중6 accepted/11 review이며 별도 미등록 raw1장도 보존했다.
다음은 근접 장면의 실제 지형 높이 산정/sparse-ground fallback 보강이다.
본10K 수집·학습/GPU 추론은 미실행이며 이 상태를 대량 수집 준비 완료로 해석하지 않는다.

[v28 상세 근거](261006_base5_appearance_ground_integration_evidence.md),
[명령/재개 런북](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md).

## v27 — 자산별 정지/이동 외관 비교 완료 (지면 정합 인증 아님)

추가 안전 gate: 동적 actor의 캡처요청 직전1초부터 저장완료까지 수신된 z를 검사한다.
인접 수신간격≤0.2초에서 z변화>0.10m이면 `motion_vertical_jump`로 후보에서 제외한다.
처음부터 일정 높이로 떠 있는 경우도 있어 objectList 이동 자산은 독립 검증된 높이
reference 없이 합격시키지 않는다. 현재 flat-ground 진단의 정지 실측 reference와
차이가>0.05m이면 제외한다. 이 임시 기준을 임의 slope/pivot의 절대 높이로 일반화하지 않는다.
raw는 원인 분석용으로 보존하고 audit를review/accepted_candidate=false로 기록한다.
저장 전 activation의 모든 z transition을 관측하거나 지면 mesh 접촉을 인증하는 gate는 아니다.
기존 raw 재검사에서 공중 배치11/11 제외, 높이 matched 재촬영12/12 통과.
Base5 테스트65개 통과. 새 live 수집은 이 추가 gate 구현 중 실행하지 않았다.

2026-10-06. 대표13자산×정지/이동×2반복의 **최종52조건/80 component GT**를 확보했다.
74시도/71 raw 저장/3도착실패 차단이며, 실패와 이전 표본을 삭제하지 않고 최종 pair를
별도 선택했다.52조건 모두 도착·실제 정지/이동·visibility·CollisionData 비충돌 검사 통과.
최종 종료Park는 첫timeout/재시도 성공이다. 간헐 서비스지연은 여전히 미해결이다.

- Dog1/2,Deer1,Ped_Pet1은 실제 자세/형상 변화가 있어 정지+이동 phase 유지 권장.
  기존Woman2 v26의 동작 외관 근거도 유지한다.
- BabyStroller2/3,Bike3,Scooter1은 현재 template에서 이동해도 뚜렷한 자세 변화가
  관측되지 않았고 component size도 동일했다. NCAP_EPTa·rigid obstacle·차량과 함께
  **외관 수집은 정지 우선**으로 설계한다. 다른 자산/애니메이션에는 자동 일반화하지 않는다.
- object 계열 일부의 정지/waypoint 이동 높이가 약0.31–0.65m 달랐다. 이 차이를
  형상 변화로 오해하지 않고12조건을 실제 높이가 맞도록 재촬영했다. raw GT 사후변환 없음.
- 초기 NPC 진단의 spawn z 중복 +0.36m는 preparer에서 수정하고 차량8조건을 재촬영했다.
- 속도 회귀/temporal 학습 재개나 runtime의 정적 객체 정책 변경이 아니다.
  detector·tracker·planning/localization production 인터페이스는 그대로다.

근거·제한·자산별 수치:
[자산별 외관 비교 EVIDENCE](261006_base5_asset_motion_appearance_evidence.md).
원본/최종비교: `/home/luv/morai_cap/base5_v2_261006/asset_motion_report_v27_canonical/`.
일반generator 통합/근접·혼합 후속은 위 v28로 대체한다. 새10K 생성·본 수집은 미시작이다.
GPU latency/VRAM은 MR 종료 후 측정한다.
최종 Base5 테스트59개, 새 스크립트 Python compile, diff whitespace 검사를 통과했다.

## v26 — 동적 외관·자연 속도 유지/뒤쪽 출발 live 검증

2026-10-06. 기존 생성기는 좁은 거리cell 내부에 시작점과 전체 경로를 넣기 위해
`speed=min(nominal,path_length/6s)`를 사용한다. 이번에는 **캡처 목표 위치를 고정하고
start=target-v×예상시간×진행방향**으로 뒤쪽 출발시켰다. 속도를 늦추지 않고 실제 위치
오차≤0.30m를 확인한 다음 캡처하는 opt-in 진단 경로를 구현했다.
진행 반대쪽이라는 뜻이며 항상 Ego 후방에 놓는 것은 아니다.

같은 Woman2/같은 목표 위치·Ego pose·진행방향에서 속도4종×3반복을 수행했다.
초기12회는10성공/2도착미확인으로 저장 차단했고, 실패2조건만 완료 resume으로
재시도하여 **총14시도/12유효 raw 캡처**가 됐다. 원래 실패를 삭제하지 않았다.
12장 모두 실제 이동·endpoint 여유·visibility/거리·CollisionData 검사를 통과했다.
raw의 파일 유효성/배치 진단이며 detector 개선·학습효과 인증은 아니다.

| 설정속도m/s | 캡처 직전 관측속도 중앙값m/s | 뒤쪽 출발 거리m | 로드→캡처요청 중앙값s |
| ---: | ---: | ---: | ---: |
| 0 | 0 | 0 | 3.784 |
| 0.35 | 0.350 | 1.75–1.925 | 5.753 |
| 1.1 | 1.106 | 5.50–6.05 | 6.326 |
| 2.2 | 2.214 | 11.00–12.10 | 6.439 |

관측속도는 수신시간 기반 위치 변화의 제한적 근거이며 공식 속도GT가 아니다.
목표도착 전에 대기하므로 표의 동적시간이 준비시간 자체라는 뜻은 아니다.
기존47장 로드→캡처요청 시간 중앙값3.685s/최대8.515s,
yaw42장 중앙값3.734s/최대8.549s였다. **6초는 고정 실측시간이 아닌 여유 budget**이다.
wall time을 simulation time과 같다고 가정해 수학적으로만 맞추지 않고 실제 도착 gate를 쓴다.

### 외관에 대한 관측과 제한

- 동일 자산 정지표본에서는 다리가 모인 자세, 이동표본에서는 벌어진 다리/팔·굴곡진
  무릎 자세가 실제 RGB crop에 관측됐다. 특히2.2m/s표본은 큰 다리 벌어짐이 확인됐다.
- 캡처GT 로컬 x-size: 정지0.352–0.437m, 저속0.453–0.781m,
  1.1m/s0.529–0.807m,2.2m/s0.598–1.069m. 동작으로 형상/OBB가 변한다는 근거다.
  3표본의 애니메이션 phase는 동기화하지 않았으므로 속도와 보폭의 단조 관계,
  모든 자산의 자동Walk→Run 전환을 인증하지 않는다.
- 생성기의walking/running tag는 속도 설정 분기였다. explicit animation 선택을
  검증한 것으로 해석하지 않는다. 원본 template의anim/behavior 값은 유지했다.
- 캡처 직전 Object 위치의 목표 오차≤0.297m. raw geometric GT를 calibration과
  정지Ego pose로 map에 복원한 목표 오차는 이번 표본 최대0.295m였다.
  capture 호출 시각을 정확한 센서 저장시각이라고 가정하지 않았다.
- 뒤쪽 출발과 도착 확인은 구현 가능하고 자연 속도 외관을 유지하는 데 유효했다.
  다만2회 도착미확인의 최초 상세 trace가 없어 원인을 확정하지 않는다.
  새 실패에는 closest error/trace를 저장하도록 추가했다.
- Park 사전probe10.2ms 성공, 각종료는 첫timeout/재시도 성공이다.
  간헐 EventCmd 지연은 여전히 본 수집 위험이다.

근거: `/home/luv/morai_cap/base5_v2_261006/motion_probe_v26/`,
`motion_capture_v26/`, `motion_report_v26_final/report.json`,
`motion_report_v26_final/appearance_contact_sheet.png`.

### 다음 적용 계약

동적 family는 목표 GT 거리/방향을 정한 후 자연 속도와 전체 경로의 지면·충돌 검사를
유지하며 뒤쪽 출발을 적용하는 방향으로 개선한다. 전체 경로를 target의 좁은 거리cell
안에 넣어 속도를 낮추는 조건은 이 방식에서 분리해야 한다. 경로 ROI/충돌 검사는
완화하지 않으며, 늦어서 목표를 이미 지나쳤으면 GT를 조작하지 않고 실패/재배치한다.
목표 도착 확인은 이번 `motion_probe`에만 opt-in이다. **일반10K 생성 default로 전환하거나
모든 animal/복합 자산에 통합 완료한 것은 아니다.** 보행자·복합·동물과 근접cell 소량
확장검증 후 일반 생성기/audit/GT 변환 계약을 함께 수정한다.
GPU latency/VRAM 측정 및 새10K/본 수집·학습은 아직 하지 않았다.

## v25 — yaw42조건 완료 / GPU 속도 측정 대기

2026-10-06. 재개 전 정지 상태의 gear-only Park probe는22.4ms에 성공했다.
남은 `straight_front +10°`, `curve_left -10°`를 원본과 같은 NPC payload로 재로드했다.
두 장 모두 고정 위치/정지 gate·실제 GT/visibility·CollisionData 검사를 통과했다.
이제 **유효42/42조건, 충돌 없는42조건**이다. 이동한 원래4회/저장 전 차단2회는
실패 이력으로 보존하며 비교에서 제외했다. 최초42회+재시도4회+최종2회로 총48회 시도,
raw46장 중 유효한 paired 조건42개다. 본 학습 quota에 넣은 것은 아니다.

| yaw 절댓값 | 실제 GT center ROI 포함 | 해석 |
| --- | ---: | --- |
| 0° | 6/6 | 모두 포함 |
| 3° | 12/12 | 양 부호 모두 포함 |
| 10° | 12/12 | 미완료2조건까지 완료 |
| 15° | 10/12 | 좌/우 곡선 각1조건 제외 |

직선·인접 차선28조건은 모두 포함, 곡선14조건은12조건 포함이다.
유효42장 중 camera 근거가 있는0-point GT는9장이다. 전체 대회 분포/AP는 아니다.
재집계한 경계 stress GT40개의 증강 유지율은 current RTS92.19%, scale 제거94.61%,
rotation 제거96.92%, translation만100%다. 기존 GT75개의99.65% 결과는 동일하다.
MORAI·GPU 모델을 동시에 돌려 추론 속도를 측정하지 않았다.

최종 종료Park은 첫3초 timeout 후 재시도 성공(총3.317초)했다. 재개 전 probe 성공과
최종gear1 응답은 확인했지만 **EventCmd 지연 원인 해결·장시간 무고장 인증은 아니다.**
다음은 MORAI 종료 후 production30m 대비100m 후보의 GPU latency/VRAM 비교,
정적NPC runtime 이동과 Park 간헐 timeout의 본 수집 gate 보강이다. 새10K/본 수집은 미실행이다.

근거: `/home/luv/morai_cap/base5_v2_261006/yaw_report_v25.json`,
`park_preflight_v25.json`, `yaw_remaining_capture_v25/`.

## v24 — 실제 yaw별 ROI 검사 / 당시 일부 재촬영 대기

2026-10-06. 직선 전/후방·좌/우 인접 차선·좌/우 곡선6개 배치에서
Ego XY와 NPC scenario payload를 고정하고 yaw만0/±3/±10/±15°로 바꿨다.
요청42회 raw 저장은 성공했으나 **runtime NPC가 움직인4회는 paired 비교에서 제외**했다.
재시도4회 중2회 복구/2회 저장 전 차단으로 **유효40/42조건**이다.
마지막 Park 응답2회 timeout으로 추가 live를 중단했다. 본 수집 승인은 아니다.

| 도로 접선 대비 Ego yaw | 실제 GT center ROI 포함 | 비고 |
| --- | ---: | --- |
| 0° | 6/6 | 직선·인접 차선·곡선 모두 포함 |
| ±3° | 12/12 | 모두 포함 |
| ±10° | 10/10 | 예정12조건 중2조건 미완료; 전체 통과로 쓰지 않음 |
| ±15° | 10/12 | 좌/우 곡선 각1조건 횡30m 밖 |

모델 LiDAR-frame ROI는 종±100/횡±30m다. 실제 curve_right +15°는
GT center=(71.81,-33.63)m, curve_left -15°는(83.25,34.54)m다.
각5 LiDAR point가 있었지만 model point ROI 안에는0개였다.
직선·인접 차선은 검증된27조건 모두 center ROI 안이었다.
**이 비율은 골라낸6개 stress 배치의 포함률이지 전체 대회 도로 포함 확률/AP가 아니다.**

- MR Ego heading-1.347°로 요청 yaw를 확인했다. 최대 오차0.108°.
  센서 GT 자체에는 heading 보정을 다시 적용하지 않았다.
- 포함 기준은 실제 LiDAR Instance center다. 학습 ObjectRangeFilter는 strict XY center,
  inference post-center filter는 XYZ다. 전체 BBox가 안에 들어간다는 의미는 아니다.
- 유효40장 중9장은 box 내부 LiDAR point0이지만 실제 대응 camera crop 근거가 있다.
  이는 lidar1_camera0 정책의 필요 근거이지 camera-only detector 성공 증거는 아니다.
- 일부 NPC는 scenario 속도 설정0인데 runtime 이동이 관측됐다. 이동 표본에서
  Object_topic 위치와 GT 위치가 함께 변했다. 원인은 아직 미확정이며 yaw 효과로 해석하지 않는다.
  yaw probe gate는 설정 위치 XY 오차>0.25m 또는 캡처 전후 이동>0.10m를 차단/REVIEW한다.
  일반 본 수집의 정적 NPC도 이 위험을 추가 검증해야 한다.
- 신규 생성 metadata에는 road-frame 의미 위치와 Ego-frame 관측 위치를 함께 기록한다.
  원본 LiDAR/camera/GT를 도로 접선 기준으로 회전시키거나 모델 ROI를 바꾸지는 않았다.
- 현재 학습 증강(R±0.1rad, T std0.2m, S0.95–1.05; RTS 순서)의 CPU
  5,000 draw 검사에서, 처음 안에 있던 stress GT38개의 평균 유지율92.65%다.
  scale 제거95.19%, rotation 제거96.76%, translation만100%다.
  기존48조건의 GT75개는99.65% 유지됐다. 전체 corpus 비율이 아닌 표본 민감도이며,
  증강 기본값을 즉시 제거하지 않고 신규10K 생성 후 거리/yaw cell별 탈락률을 검사한다.
  설치된 MMDet3D box 회전/이동/scale CPU 수치와 RTS 계산을 대조했다.
- 원래 offline audit의 ROI 오류2개는 의도적인 범위 밖 진단이다.
  `OFFLINE_FAIL` 원본을 그대로 보존하고 report에서 해당2개만 기대 오류로 구분한다.
  이 장면을 일반 학습 quota의 accepted 후보로 세지 않는다.

근거: `/home/luv/morai_cap/base5_v2_261006/yaw_report_v24.json`.
원본 `yaw_probe_v24/`, `yaw_capture_v24/`, `yaw_retry_capture_v24/`를 보존했다.
미완료는 `b5yaw_straight_front_p10`, `b5yaw_curve_left_m10`이며
`yaw_remaining_manifest_v24.json`에2조건만 준비했다. Park/연결 확인 없이 재개하지 않는다.
구현·재현 명령은 [동기 캡처 런북](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md)의 yaw 검사 절 참조.

## v23 — 동작 타이밍·48조건 분산검증·GT 정책 결정

사용자가 요청한1–3단계를 수행했다. **새10K 생성·본 캡처·학습은 하지 않았다.**
현재 default는 왕복 없는 일방향+appearance profile이다. 이전v18d의 왕복10K는 그대로
보존하며 신규 수집에 사용하지 않는다. v21 저속 결과는 아래 이력으로 남긴다.

### 1. 움직임과 캡처 타이밍

- 넓은 거리 cell은0.5–16m 일방향 경로를 허용한다. 종류별 nominal cap 후보는
  걷기1.1/달리기2.2/Animal1.2/자전거·스쿠터2.5m/s이며,
  실제 설정은 `min(cap, 구간길이/6초)`다. 좁은 cell에서는 계속 느려질 수 있다.
  초저속만으로 모든 동작 quota를 채우지 않는다. 속도 label/temporal 학습은 하지 않는다.
- 거리 cell 전체 유지 검사는 선분의 정확한 최소·최대거리로 계산한다. ROI/방향,
  swept clearance, 실제 annotation 및 CollisionData gate는 유지한다.
- 실제 관측속도: 걷는 사람1.182m/s, 달리는 사람2.107m/s, Scooter1 1.917m/s.
  수신시간 기반 위치 변화이며 공식 속도GT·애니메이션 인증이 아니다.
- 캡처 직전0.3초 trace에서 실제 이동을 확인하고, 저장 전후 waypoint 끝점까지
  최소0.1m 여유가 없으면 REVIEW다. 정지·배경에는 적용하지 않는다.
  저장 시각을 정확히 아는 검사가 아니라 보수적인 전후 구간 검사다.
- 추가 actor도 같은 검사를 적용한다. 초기48장에서는 primary live gate였으나
  남긴 trace로 secondary2건을 별도 재검증했고 모두 통과했다. 최종 all-actor gate는
  두 Ped_Pet1이 움직이는 장면1회로 live/resume까지 확인했다.
- 이동이 확인되지 않은 실패에는 새 코드부터 별도 failure telemetry를 남긴다.
  첫 실패의 endpoint 직접 trace는 없으므로 원인을 확정하지 않는다.

### 2. 분산 pilot 및 제한적 재배치

| 패밀리 | 독립 조건 | 범위 |
| --- | ---: | --- |
| 고주로 | 12 | 전·후방 각각6개 거리 cell,6–99m |
| 로터리 | 4 | 이번 표본은10–20m 중심, 전 범위 인증 아님 |
| 정적 차량 회피 | 4 | 접근·진입·좌/우 alongside |
| 실체 장애물 | 12 | 회피4조건+범용 전/후/좌/우 및60–99m |
| 사람·동물·복합 객체 | 14 | 걷기/달리기/정지,Dog1/2·Deer1·dummy·유모차·탑승자·반려동물 |
| 빈 배경 | 2 | 정상 negative |
| 합계 | 48 | 전체 카탈로그 전수 인증 아님 |

첫48회는 **46후보/1검토/1실행실패**다. 후방80–99m DEFAULT_MPV는 runtime/annotation
모두 미관측이었다. 같은 자산·cell로 재배치한96.786m 장면은3point로 통과했다.
자산 자체 미지원으로 결론내리지 않는다. Ped_Pet1은 움직임을 확인하지 못해 센서
저장을 진행하지 않았다. 첫 시도9.53초와 명목6초 경로는 조기 endpoint 가능성을
시사하지만 직접 증거는 없다. 같은 객체·cell의 다른 일방향 배치는 통과했다.

두 재배치+최종 all-actor 반복1회까지 총51회 시도/50개 raw 캡처다.
**독립 조건은48개**이며 반복을 데이터 다양성 증가로 세지 않는다. 원래 실패·검토를
삭제하지 않고 안전 종료한 대체본만 canonical selection에 넣었다.

- 원거리 실제 primary: 전77.178m/4point,97.056m/10point;
  후63.616m/6point,96.786m/3point.
- alongside primary annotation gap: 정적 차량0.127/0.200m,
  장애물0.364/0.390m. 해당4조건은 CollisionData 이력에 non-Ego 충돌이 없었다.
  Ego proxy와 annotation gap이지 실제 collider 간격 측정은 아니다.
- 종료Park은 각 검증 세션 첫 요청 성공. 대표48장 종료17.7ms,재배치9.1ms.
  완료된47개 raw의 hash/mtime·ledger 불변 resume 확인; 실패1개를 자동 재캡처하지 않았다.
  최종 all-actor 세션도 완료 resume 통과했다. 장시간 무고장 보장은 아니다.

### 3. GT·변환·수집 준비 결론

학습 준비 기본은 **lidar1_camera0**로 정했다. 원본 LiDAR Instance geometric box를
보존하고,≥1LiDAR point 또는 실제 대응 camera Instance의 모델 crop 근거(≥16pixel,
extent≥4×4)를 요구한다. 같은 부모·class의 복합 component를 구분하지 못하면
camera-only positive를 억지로 배정하지 않는다. raw에는 제외 component도 보존한다.
primary 근거가 없는 장면을 빈 배경으로 바꾸지 않는다.

| GT 정책 | 대체본 선택 후 usable 조건 | 유지 GT |
| --- | ---: | ---: |
| lidar5 | 38 | 50 |
| lidar1 | 46 | 69 |
| lidar1_camera0 | 48 | 75 |

실제0-point camera-only positive5개(Obstacle4/Animal1)를 보존했다.
이는 camera 근거 정책이 작동한다는 증거이며 AP 개선·최적 threshold 증명은 아니다.
변환 제외 component는Obstacle1/Animal1/Pedestrian3개이며 raw/audit에 남는다.

최종48조건→train42/val6/test0,GT Vehicle26/Pedestrian20/Obstacle23/Animal6.
7D single-frame/no velocity target이다. 4카메라 파일·실제 해상도·LiDAR4feature·finite
보정행렬·양의 box 크기·class index CPU 검사 모두 통과했다. 이번에는 MMDet3D
training forward나 모델 성능평가를 하지 않았다. 기존production3-class E25는 그대로다.

최초48회 평균5.255초/p50 4.944초/p95 8.054초. 최종 선택본 평균5.168초.
이번 모든 raw 약240.7MiB,최종 선택본231.2MiB. 단순10K 환산은 약14.4–14.6시간,
47GiB 전후 raw다. 재시도·MORAI 장시간 저하·파생 캐시는 별도이며 소량 추정이다.
현재 디스크 여유269GiB로 공간은 충분하지만 과거3-frame 수집 속도로 예상하면 안 된다.

**준비 판단:** 제한적인48조건 수집·GT·변환·모니터링 경로는 통과했다.
다음은 새 기본 정책으로10K 시나리오를 별도 생성하여 전수 quota/배치 audit하는 단계다.
512회 탐색 한도 안 모든 희소 cell이 채워지는지는 미검증이며 실패 시 quota를 몰래
축소하지 않는다. 전체pool·로터리 전체거리·장시간 서비스 안정성과 모델 일반화는 남는다.
10K 생성/본 수집은 다음 사용자 지시 전 시작하지 않는다.

증거 prefix `/home/luv/morai_cap/base5_v2_261006/`:

- `readiness_pilot_v22/offline_audit.json`:48개 OFFLINE_PASS
- `readiness_capture_v22/smoke_report.json`, `readiness_report_v22.json`:첫 시도 그대로
- `readiness_retry_pilot_v22/`, `readiness_retry_capture_v22/`:2조건 재배치
- `all_actor_capture_v23/smoke_report.json`:최종 all-actor live/resume
- `readiness_selected_v23.jsonl` 및 `.provenance.json`:48개 canonical,원본불변
- `readiness_selected_report_v23.json`, `gt_policy_readiness_replaced_v22.json`
- info: `artifacts/datasets/base5_readiness_selected_v23/`

단위검증43개. Dashboard `http://127.0.0.1:8075`는 첫48회 상태를 그대로 보여준다.
재배치 후48개 최종 선택 결과는 selected report를 본다.
Session schema5는 all-actor motion 계약이다. v21/schema3,v22/schema4 output에 현
runner로 resume하면 계약 불일치로 거부한다. 과거 세션을 새 계약으로 자동 승인하지 않는다.

## v21 — 왕복 제거·저속 일방향 smoke / 본 수집 미시작

사용자와 논의한 일방향 대안을 구현하고 실제 움직이는6조건을 검증했다.
`one-way-slow-v1`은 **현재 heading 방향으로2개 waypoint만** 사용하며 왕복/방향반전/
respawn/teleport를 넣지 않는다. 초기 heading과 이동방향을 맞추고,0.5–4m 구간 전체가
추정 geometric 거리 cell 안에 머물도록 선택한다. Ego/다른 actor swept clearance와
ROI/방향, 실제 CollisionData·annotation gate는 유지했다. Ego는 정지·제동 상태다.

좁은 cell에서 너무 느려질 수 있어12초 준비 여유 프로필과6초 프로필을 순차 비교했다.
설정 속도는 `min(종류별 nominal cap, 구간길이/준비여유)`이며 cap은
Animal0.25/걷기0.35/달리기0.6m/s 후보다. raw speed 필드에는3.6배 값을 넣는다.
실제 이동은 검증했지만 객체별 speed 필드 단위·동작 애니메이션은 독립 인증하지 않았다.
특히 requested walking/running quota를 실제 걷기/달리기 동작으로 확정하지 않는다.

| 조건 | 요청거리/방향 | 12초 프로필 | 6초 프로필 |
| --- | --- | --- | --- |
| worker_m_1 | 좌5–10m | 통과, nominal0.067m/s | 통과,0.133m/s |
| Dog2 | 좌15–29m | 센서 근거 부족 검토,0.25m/s | 이번 반복 미대상 |
| BabyStroller2 | 우3–5m | 통과,0.117m/s | 통과,0.233m/s |
| Ped_Pet1 | 전15–30m | 통과,0.333m/s | 이번 반복 미대상 |
| Bike3 | 전5–10m | 통과,0.192m/s | 통과,0.35m/s |
| BabyStroller2 | 전3–5m | 통과,0.05m/s | 통과,0.10m/s |

합계10회 실제 캡처:9회 후보/1회 Dog2 가시성 검토/0실행실패.
**서로 독립10개 조건이 아니라6조건+동일4조건 속도 A/B 반복**이며 수량 quota로 중복 합산하지 않는다.
전부 기다리는 동안≥0.05m 실제 이동을 확인했다. 종료 Park은 두 세션 모두 첫 요청에
4.1/5.7ms 성공. 세션평균5.294/5.048초, raw30,734,186/20,386,467bytes.
완료 resume에서 raw hash/mtime·ledger 불변을 확인했다. Info 변환은 각각 train5/val0
GT13, train4/val0 GT9이며 학습·AP 평가를 하지 않았다.

6초 프로필 실제 component 거리/point:
worker9.508m/37point; 우유모차3.691/3.234/3.162m·134/110/8point;
Bike3 5.413/5.377m·119/188point; 전유모차4.909/5.078/5.115m·106/102/28point.
전유모차 일부 component는5m를 넘으며 기존±0.5m annotation tolerance 내 통과다.
Dog2는24.084m·0point이고 유효 camera 근거도 없어 검토 유지다. 거리·point 기준을
완화하거나 정답을 임의 추가하지 않았다. 전유모차 raw CAMERA_1도 눈으로 확인했다.

마지막 전유모차의 endpoint를 캡처 종료 이후 read-only3초/150메시지 관측:
시작점에서0.599991m 이동, 예정 endpoint 오차0.000021m, 관측 이동span0m였다.
이 자산은 끝에서 멈췄으며 복귀/loop가 관측되지 않았다. 모든 container 인증은 아니다.

**새 생성기 default는 일방향/6초 여유로 변경**했고 기존 왕복은 명시적 역사적 옵션으로
보존한다. 기존v18d 10K manifest는 왕복 계약인 구버전 후보이므로 새 기본 수집에 그대로
사용하지 않는다. 이번에는19개짜리 offline pilot 두 개만 생성했고 10K 재생성·캡처는 하지 않았다.
초근접 전유모차는128회 배치 탐색에서 미충족,1024회 한도 안289retry로 채웠다.
부분 실패v21은 보존한다. 이 희소한 배치의 전수10K 충족 가능성은 아직 검증하지 않았다.

증거 공통prefix `/home/luv/morai_cap/base5_v2_261006/`:

- `one_way_pilot_v21b/`, `one_way_pilot_v21c/`: 각각19개 OFFLINE_PASS
- `one_way_capture_v21b/smoke_report.json`, `one_way_capture_v21c/smoke_report.json`
- `one_way_motion_comparison_v21.json`, `one_way_endpoint_v21.json`
- info: `artifacts/datasets/base5_one_way_v21b/`, `base5_one_way_v21c/`

단위검증38개 통과. 이 결과는 왕복 없이 거리 유지·실제 이동·캡처·resume이 가능하다는
근거다. 모두 초저속으로10K를 수집하자는 뜻은 아니다. 더 빠른 사람/탑승자의 다양한
동작 외관, readiness 지연 시 endpoint 도달, 전체pool quota는 본 수집 전 남은 gate다.
준비가 늦어 motion 확인 전에 endpoint에 도착하여 움직임이 없으면 실패시킨다.
motion 확인 후 저장 직전까지 움직임을 유지하는지 runtime 보장은 아직 별도 보강 대상이다.
이번10회는 post-capture Object pose를 대조해 endpoint까지0.252–3.224m 남았음을 확인했다.
끝점에 멈춘 상태의 캡처가 관측되지는 않았으나 이 소량 증거로 모든 지연 상황을 보장하지 않는다.

## v20 — 종료 Park 요청 최소 수정 / 소량 재검증

이번 요청은 v19의 Park 응답 문제만 대상으로 했다. 왕복 motion/거리 quota/GT 정책은
변경하지 않았다.10K 수집·학습은 하지 않았다.

현재 정지 상태에서 독립 요청은 기어만(option2)15.5ms, 제어모드+기어(option3)10.5ms로
모두 P 응답이었다. 이어 동일 one-load/capture 수집기로 비교했다.

| 종료 요청 | 장면 | 첫 요청 | 최종 P 응답 |
| --- | --- | --- | --- |
| option3: control mode+gear | 빈 배경 | 3초 timeout | 두 번째 성공, 총3.312초 |
| option3: control mode+gear | Dog2 동적 | 3초 timeout | 두 번째 성공, 총3.308초 |
| option2: gear only | 빈 배경 | 성공 | 11.3ms |
| option2: gear only | Dog2 동적 | 성공 | 3.9ms |
| option2: gear only | 빈 배경 반복 | 성공 | 7.4ms |

종료는 **P 기어만 변경**하도록 최소 수정했다. 로드 직후 external control/Drive 확인은
기존 option3 유지다. 종료시에 이미 확인한 control mode를 다시 설정하지 않는다.
요청당3초·최대2회·제동 유지·응답 gear1 확인·실패 중단은 모두 유지한다.
shutdown evidence에 실제 request_fields를 추가했고 잘못된 gear 응답도 여전히 실패다.

현재 소량3회에서 첫 요청 성공을 확인했으므로 종료 수정 후보는 통과다. 다만
비무작위 순차 A/B이며 MORAI 내부 응답 누락 원인은 미확정이다. 기존 v19의 실패를
성공으로 수정하거나 unsafe session을 재개하지 않는다. 장시간 안정성 보장은 아니다.
P는 서비스 응답으로 확인했으며 EgoVehicleStatus에 gear 필드가 없어 독립 gear topic
대조는 수행하지 못했다. 두 Dog2 캡처의 visibility review는 별개로 유지했다.

증거 공통prefix `/home/luv/morai_cap/base5_v2_261006/`:

- `park_response_comparison_v20.json`, `park_rosbridge_log_v20.log`
- `park_probe_v20_gear_only.json`, `park_probe_v20_combined.json`
- `park_baseline_capture_v20/`, `park_dynamic_capture_v20/`
- `park_gear_only_capture_v20/`, `park_gear_only_dynamic_v20/`, `park_gear_only_repeat_v20/`

독립 probe는 `scripts/probe_base5_park.py`: 정지·fresh Ego·제동 subscriber를 확인하고
한 번의 bounded Park 요청만 보낸다. 시나리오 로드/캡처/가속 없음, 증거 덮어쓰기 거부.
단위검증36개 통과. 다음은 사용자와 논의한 일방향 저속 motion 대안 검증이며 아직 구현하지 않았다.
프로토콜에서 option2=gear, option1=control mode, gear1=P는
[MORAI 공식 규격](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R1.0/ros-1)을 참조한다.

## v19 — 동일 cell 보충 구현 / 종료 Park 미확인으로 live 중단

사용자 요청대로10K 캡처·학습은 하지 않았다. v18 후보 생성기는 유지하며 소량 검증만 수행했다.
`base5_replenishment.py`는 **가시성만 실패한 review**에 한해 같은 asset/spec/split을
유지한 재배치를 만든다. 거리·방향·motion·미션 cell을 바꾸거나 GT tolerance를 완화하지 않는다.
최대16개 cell, cell당1–4회, 전체64회 이하이며 첫 통과 뒤 그 cell은 종료한다.
충돌/객체 누락/거리 실패가 섞인 review는 자동 보충 대상이 아니다. 원본과 실패 raw는 보존한다.
provenance·동시 실행 lock·STOP·child 실패 즉시 중단을 포함한다.

실제 Dog2 좌측15–29m cell에3회 한도의 재배치를 준비하고 **첫1회만 캡처**했다.
Dog2 LiDAR2point로 capture audit 통과,4.861초였다. 동일 spec/data ID50300003/train split이
세 후보 모두 유지됨을 검사했다. 그러나 종료 Park 요청은3초씩2회 모두 timeout이었다.
40Hz 제동은 두 요청 동안 유지했으나, Park 응답을 입증하지 못했으므로 전체 보충은
`halted`, child는 `halted_exit_safety`로 중단했다. 나머지2회는 로드하지 않았다.
**캡처 품질 통과와 안전한 세션 종료는 별도다. 보충 완료/안전 종료 통과로 보고하지 않는다.**

- 증거: `.../replenishment_v19/report.json`,
  `.../attempts/b5v2_00024_r01/shutdown_safety.json`, captures 내 raw/audit/telemetry.
- 완료 child 재실행은 ROS 없이 raw34파일+ledger hash/mtime 불변을 확인했고,
  종료 실패 상태를 complete로 지우지 않는다. 실패한 보충 세션의 자동 재개는 거부한다.
- 사후 read-only3초: Ego150/Object149/Collision150개, Ego velocity.x≈0.00000153m/s.
  전체 topic 연결 단절은 관측되지 않았다. EventCmd 응답 누락 원인은 미확정이다.
  다음 live는 MR 제어/기어 확인 후 새 소량 세션으로 검증해야 한다.

### GT 정책 비교 — 최종 모델 성능 결정 아님

v18d16장+추가3장, AI 재검증은 같은ID를 대체하여19개 unique scene을 비교했다.
종료 실패 Dog2 보충은 아래 주 비교에 넣지 않았다.

| 정책 | 사용 장면 | 유지GT | 제외component |
| --- | ---: | ---: | --- |
| LiDAR≥5point | 17 | 22 | Pedestrian2/Animal1 |
| LiDAR≥1point | 18 | 26 | 0 |
| LiDAR≥1 OR 실제 모델crop 카메라 근거 | 18 | 26 | 0 |

세 정책 모두 센서 근거 없는 원래 Dog2 scene은 review로 유지한다. 5point는 한 장면을
추가 제외하며, 나머지 사용 scene에서도 일부 복합 객체 component를 제외한다.
이 표는 quota/visibility 진단이고 AP 평가가 아니다. 표본에는 유효한 camera-only positive
증거가 없어 camera OR 정책의 추가 이득을 입증하지 못했다. 현재 provisional default
`lidar1_camera0`를 유지하되 최종 본 학습 승인으로 해석하지 않는다. raw annotation은
모두 보존하므로 이후 정책 교체에 재수집이 필수는 아니다.

`report_base5_gt_policies.py`가 ledger SHA·scene별 사용/검토·class/거리/방향을 기록한다.
증거: `.../gt_policy_comparison_v19.json`. 실제 info 변환도 v18d 대표16장으로 세 정책
모두 수행했다: lidar5 train11/val3 GT16, lidar1 및 OR train12/val3 GT19.
산출물 `artifacts/datasets/base5_v19_gt_policy_smoke/`, 학습 미실행.
단위검증35개 통과. production detector/threshold/planning/control/localization은 변경하지 않았다.

## v18 — 동적 거리 수정·live 복구 / 본 수집 미시작

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
