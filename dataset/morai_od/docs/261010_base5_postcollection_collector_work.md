# Base5 종료 후 수집기·새 환경 배치 작업

2026-10-10 / 수집기 작업 완료 EVIDENCE. 최종 Sunny13 5K와 새 실험/생성 경로를 구분한다.
후속 작업 복구의 시작점이며 GPU 모델/crop/BEVPool 실험은 이 작업에 포함하지 않는다.

## 보존한 현재 데이터

- `~/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_sunny13_flexible_v140`
  최종 5,000승인/부족0, 기존 유효4974+새26다. 최종 selected ledger만 사용한다.
- 원래 Sunny13 source SHA `89b053e7332e20f284c2de28d2f2365a3a9cf157d017c628481d9715ec5a1a77`와
  원본 capture/runtime/journal/참조를 보존한다. 기존 generator/collector 파일을 직접 수정하지 않았다.
- 새 분석/실험 root는 `~/morai_cap/base5_postcollection_261010`이다. 새 결과를 기존 수집에 import하지 않는다.

## 완료한 전수 읽기 전용 분석

`report_base5_postcollection.py` → `observed_v140.json`.
status의 누적 active는13.6575시간이다. 부모 import 누적시간들을 더하지 않는다.
하드링크/중첩 root를 중복 집계하지 않은 native timer는5,515 load interval이다.
resume 후 timer가 이전 segment를 덮어쓰고 interpreter/부모 QA가 빠지는 한계가 있으므로
native timer 합을 전체 wall이라고 부르거나 active와의 차이를 전부 제거 가능한 overhead라고 부르지 않는다.

| native 단계 | timed load당 초 |
|---|---:|
| pose/map 지지 확인 |2.064|
| 서비스/제어 준비 |1.917|
| 파일 write/안정화 |1.159|
| Ego 정지 증거 |0.342|
| provenance/충돌/센서 audit |0.080|

전체5K raw component11,372 중 현재320crop/ROI 조건의 유효GT10,959,
제외413(3.63% GT)다. 풍부한 단서8,272(75.48%), LiDAR1–4점/camera 조건미확인697(6.36%),
camera-only586(5.35%)다. 이는 장면 실패율/모델 정확도가 아니다.
train에122자산 모두 존재하며 분석용 풍부한 단서5개미만은
OBJ_Hyundai_Xcient_Mixer(3/3), OBJ_Kia_K5(4/4), OBJ_Kia_Niro(4/4)다.
각 분모는 train 유효 component GT이며 최종 crop 결정 후 다시 계산한다.

범용 regular/dense 유효 primary1690/보조4603 parent 배치에서2점 크기군은
primary806(47.7%)/보조2357(51.2%)다. 가시 보조parent2191이며
복합component를 포함한 해당 크기군 유효GT는 primary874/보조2313이다.
2점=작은 화면 객체가 아니고 비용/차종 pool 조건이 함께 작용한다.
현재 점수제 폐지·작은 자산 제외·보조가중치 변경을 뒷받침할 정도의 편중 근거는 없다.
기존 성공분과 다음환경 primary 할당/예산 비율을 유지한다.

## 새 환경 생성 구현

- `base5_spatial_diversity.py`, `prepare_base5_fresh_environment.py`, `prepare_base5_fresh_suite.py`.
- 원래 구성표/primary/거리·방향/미션phase/split은 보존하고 환경별 geometry seed를 새로 만든다.
  config seed만 바꾸거나 기존 payload를 복사하는 방식이 아니다.
- 범용 route_general은 허용 도로/구역/physical-link split interval에서 연속 arc를 뽑는다.
  실제 final XY 높이/footprint 검사, NPC 실제경로, 횡단축, 정지·실제 이동 정책은 기존 것을 사용한다.
- 처음 좁은 strata만 적용한 pilot은 원거리 곡선/전용구역 근처에 unfillable cell을 만들었다.
  거리·방향별 primary 배치 가능성 proxy를 먼저 검사하고 compatible interval에 quota를 배분한다.
  초기12m local window,64회 후24m,256회 후48m로 bounded 확대한다.
  전체지도 재추첨으로 어려운 quota를 몰래 버리지 않으며 사용 window/arc/final XY를 기록한다.
- 같은 논리slot이 다른환경에서 새geometry가 된다. unique geometry group/token과 물리link split을
  구분한다. final-support link/인접100m시야 중복을 별도 감사하며 완전한 지리적 held-out을 주장하지 않는다.
- `run_base5_capture_campaign_spatial.py`와 `run_base5_fresh_batch.py`가 quality 재배치에도 저장된
  공간 plan을 연결한다. 기존 Sunny13 runtime/출력으로 새adapter를 강제resume할 수 없다.
- 생성 입력/spec/hash는 새root에 저장한다. 실제 UI 날씨/시간 변경과 본수집 승인은 별도다.
  진단용 v141/v141b/v141c 중간생성은 보존하되 최종다음환경 입력으로 바로 쓰지 않는다.

## 시간 최적화 실험 구현과 경계

### clean 재측정·장기 회귀 완료

서비스 확인 순서를 유지한 후보의 clean baseline(A2/A3) 평균389.488초 대비
optimized(F1/F2) 평균301.647초, **22.553% 단축**이다. 모든 반복64/64승인/검토0이다.
별도200장 장기 검증은955.731초/200승인/검토0이고 actual animated8장도226.109초/8승인이다.
200장 및64장 완료resume은 ledger/raw 불변·추가촬영0이다. 이 값은 같은payload의 부모전체active이며
완료resume 검증에 걸린0.4–0.7초를 재촬영시간으로 사용하지 않는다.
실제 동적 준비는 단축하지 않았고 본5K는 아직 이 runtime으로 수집하지 않았다.

| 같은64 payload 반복의 단계 | baseline 초/load | optimized 초/load |
|---|---:|---:|
| pose/map 확인 |1.836|1.506|
| 서비스/제어 준비 |1.920|1.252|
| session/ROS/preflight |0.138|0.035|
| 파일 write/안정화 |1.179|1.173|
| Ego 실제 정지 증거 |0.340|0.341|
| 충돌·센서·provenance audit |0.078|0.075|
| native timer 밖 부모 비용 |0.333|0.080|

실측 단계분해이며 pose 차이 전부를 cache의 인과효과로 단정하지 않는다.
확실한 변경은 조건부0.7초 setup 대기 조정과chunk4→16의 제한된 준비 재사용이다.
GT·파일안정화·정지검사를 빼서 줄인 시간이 아니다. 기존 최종13.6575시간과 이 pilot의7.224시간
외삽 차이를 전부22.55% 최적화 효과라고 해석하지 않는다(실패·보충·표본 구성이 다름).
200정적/8이동을4822정적/154교차로이동/24음영이동으로 가중한 예상은7.224시간,
실패/보충/새환경 여유를 둔 운영예산8–11시간이다. 확정된 full-5K 소요시간이나 통계적 신뢰구간은 아니다.
pilot 저장가중27.036GiB/5K, 보수 공간예산35GiB/5K를 별도로 둔다.

### 25K 생성 중 근접 rider 사례

`fresh_suite_final_v141/sunny15`의02629는3–5m 전방 Bicycle_Female1 quota였다.
512개 CPU 후보 중 primary 간격실패502개, 보조 간격실패6개, 거리bin실패4개로 성공이 없었다.
Ego 후륜기준3–5m에는 차체앞끝이 포함돼 collision-free 영역이 좁다. 위치 window를 더 넓히거나
자산/근접quota를 빼는 것이 해결책은 아니었다. 같은plan/자산/거리/sector/검사를 유지한
attempt602(603번째 후보)가0.348m clearance로 통과했다. 따라서 생성전용 유한예산4096개로 보완하고 후보 수/사유를 기록한다.
이는 **offline JSON 생성** 비용이며4096번 실제캡처/retry가 아니다. geometry/GT/QA 기준을 완화하지 않는다.
실패v141은보존하고 `fresh_suite_final_v142`에서 코드동결 후5환경 전수생성을 완료했다.
25,000개payload/token 중복0, original primary/거리/방향/미션phase/split drift0,
각환경4255train/745val·정적4822/이동178·122종primary floor 감사PASS다.

| 환경 | unique base XY(3dp) | unique final Ego XY(3dp) | 최대CPU 후보수 |512개 초과slot|
|---|---:|---:|---:|---:|
|sunny11|4992|4999|434|0|
|sunny15|4996|4999|603|1|
|foggy11|4993|4998|563|1|
|foggy13|4992|5000|407|0|
|foggy15|4991|4999|229|0|

매환경1882개 범용slot에 연속arc policy를 적용했다. Sunny11 범용장애물770개와사람계열830개는
각각기존98base에서770/830base로늘었고 기존base 일치0이다. 고주로740/로터리350 등은 이미
연속미션/경로 샘플링이므로 그 규칙을 그대로 둔다. base가다르다고 배경까지독립이라고 해석하지 않는다.
physical-anchor-link split충돌0이나 final support link9개가train/val 양쪽에있다(회피관측재배치).
인접100m시야의공간중복은남으며, 합성val을실전held-out rosbag 대체로사용하지 않는다.

### 추가 후보 중단 / 2026-10-10

첫 baseline64 두회 평균389.556초, static-ready+chunk16 두회 평균301.620초로22.57% 단축,
모두64/64다. delay-only chunk4는350.924초다. 이는200scene/이동/새geometry 승격 전 기록이다.
read-only service 준비를 pose 대기와 겹친 추가후보는 chunk4의64/64 이후 chunk16 e1에서
`b5v2_03511`의 `service [/Service_MoraiEventCmd] unavailable`로 중단됐다.
early probe가 이전SceneNetwork 서비스를 확인한 뒤 reload로 사라지는 경쟁 조건으로 추정한다.
scene boundary/final Park는검증됐으나 unsafe target failure를 부족분으로 자동 건너뛰지 않았다.
실패원본/32선택/로그/계약을보존하고 강제resume하지 않는다.
이 후보는 **탈락**이며 새fresh runtime의 `--runtime-overlap`도 실행 전에 거부한다.
조건부0.7초 대기만 조절하되 service 확인 순서를 유지한 기존후보로 clean 재측정/200장 검증을 완료했다.

### 새 환경 유한 보충·진입점 / 실제 신규 부족분 검증 완료

`run_base5_fresh_completion.py`는 fresh terminal batch를 native read-only로 복원·검증한 뒤
별도selection을쓴다. oldSunny13 하드코딩 driver를 재사용하여 원래output을수정하지 않는다.
family/mission/semanticpool/split/실제primary gait·moving mode별최소인원은유지한다.
자산/거리/방향/개수변경은명시delta와 adopted_manifest에남긴다. 희귀primary floor미달이면
다른자산대체를금지한다. 같은payload재촬영·knownSHA 재사용·partial/unsafe child 자동reload를금지한다.
셀당누적3회(최대5), 실행당1800초(최대3600), PAUSE/STOP/완료후resume촬영0이다.
강제3회review 이후재실행이상한을초기화하지않는 단위검사를추가했다.
static은검사유지one-shot, actualmoving은독립준비를유지하는 spatial campaign이다.
`launch_base5_fresh.py`는 qualified release/code/manifest hash, explicit MR UI 환경확인을검사한다.
한환경의본수집+finite보충을한entrypoint로묶되, 신규보충의live qualification 이전에는
`--complete`를본수집시작전에거부한다. 이번live qualification은완료했으며 실제25K는자동실행하지 않았다.

- `run_base5_capture_optimized.py`는 frozen collector AST의 명시적0.7초 setup 대기만 opt-in 조건부로 바꾼다.
  정적/빈배경+fresh post-load transport일 때만 생략, 동적은 원래 대기를 유지한다.
  fresh service proxy/검증응답/한정 재시도·매scene network reload·Park·실제정지·접촉·0.5초 파일안정화는 그대로다.
- source AST 구조가 달라지면 fail-closed. adapter/source hash를 session runner_options에 기록한다.
- `run_base5_batch_optimized.py`의 별도guard는 code/manifest/chunk/profile 변경을 거부한다.
  profile별 새output만 사용한다. 최대16scene chunk의 ROS/map 공유이지 chunk를 넘는 영구상주worker라고 부르지 않는다.
- 사전정한64정적/8이동 payload 목록은 `runtime_benchmark/registration.json`이다.
  결과/score를 보고 장면을 고르지 않았다. baseline chunk4 → delay-only chunk4 → combined chunk16
  → combined chunk16 → baseline chunk4의 반복/역순 비교를 완료했다.
- CPU 생성이 초기 baseline/첫delay 일부와 겹쳤으므로 나중의 clean 역순pair와 일치하는지 확인한다.
  20% 이상 full parent 비용 감소와 승인/안전 유지 전에는 자동승격하지 않는다.
  64scene 합격 후 별도200scene 장기/8이동 회귀와 resume 검사도 모두 통과했다.

## 새 배치 실제 MR·보충·변환 최종 결과

- 새정적64장: 최초63승인/1부족, 부모active421.379초(chunk4, pause/resume 및native quality 재배치포함).
  01521은native quality 재배치로복구됐고,04635는얇은Lane_Separation_Bar/혼합보조가시성부족이었다.
  `fresh_geometry_completion_v3`의첫새후보로04635도복구해64/64다. 원래63행·부모/실패raw·journal은보존한다.
- 새이동8장: 최초7승인/1부족. 음영Dog1의static ground 준비가검사에걸렸고
  `fresh_geometry_animated_completion_v3`의첫새후보가독립지면준비/실제walking 증거를통과해8/8이다.
  원래primary gait·moving mode별최소인원과source split을모두검사했다. 지면검사완화나static 대체가아니다.
- 정적/이동 유한보충은각1시도, parent hashes불변, terminal resume journal/선택/시간/raw불변이다.
  native batch selected row에`scenario`필드가없는경우의adopted_manifest serializer 오류와
  background의distance-bin 부재를live roundtrip에서발견해수정했다. 실제보존scenario SHA를검증하며
  초기manifest payload로잘못대체하지않는다. 같은자산이라도다른split donor가source fallback을막지않게했다.
  실패preflight v1/v2는보존했고 수정코드로강제resume하지않았다(촬영미실행).
- CPU infos64장과기존Sunny13 infos5K의compatibility merge5064/중복·group split누수0이다.
  qualification pilot은본학습/30K에포함하지않는다. 실제새날씨조건을검증한것도아니다.
- `collector_release_v142/release.json`에최종helper절대경로SHA/입력suiteSHA/profile/유한보충/실측근거를동결했다.
  5환경`launch_base5_fresh.py --prepare-only`모두PASS이며child/실제본수집미실행이다.
- `base5_fresh_dashboard.py`는같은source/parent lineage의`_completed` 선택을자동관측한다.
  parent와보충성공분을더하지않고다른환경의보충도suite에반영한다. 큰5Kstate는8KiB lineage header만읽는다.
  원래8075관측기/수집runtime은바꾸지않았다. 새8082는전체30K중Sunny13 5K만승인된상태다.
- 최종Sunny13 ledger SHA는그대로다. MR worker는종료/Park확인, production/GT/threshold/model/GPU 작업변경없다.
- 최종 `test_base5*.py` 전체 단위 검사: 554개 PASS, 73.771초. `git diff --check`도 통과했다.

## 다음 실행

- 5환경25K offline manifest/coverage/token/split/위치 감사는v142에서완료했다.
- 수집기/새배치live·200scene·실제이동·보충·resume·최종Park·CPU변환은완료했다.
- [새단일진입점런북](261010_base5_fresh_environment_runbook.md)으로UI환경확인후새5K씩수집한다.
  완료resume은상한을초기화하지않으며native QA/안전실패를무시하지않는다. 미래모든부족분의복구를보장하지않는다.
- 실제다른25K와GPU/crop/BEVPool/cache/초기화실험은미시작이다. GPU 작업전MR을꺼야한다.
  crop·투영확정후visibility/infos를다시구성하고동일조건4090 비용을검증한뒤본학습한다.
