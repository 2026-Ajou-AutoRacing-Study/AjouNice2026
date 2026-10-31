# Base5 종료 후 수집기·새 환경 배치 작업

2026-10-10 기준 수집기 작업 완료 근거(EVIDENCE)다. 최종 Sunny13 5K와 새 실험·생성 경로를 구분한다.
후속 작업 복구의 시작점이며 GPU 모델·crop·BEVPool 실험 결과는 별도 문서에서 관리한다.

## 보존한 현재 데이터

- `~/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_sunny13_flexible_v140`
  최종 승인 5,000장/부족 0장이다. 기존 유효 4974장과 새 26장을 합쳤으며 최종 selected ledger만 사용한다.
- 원래 Sunny13 source SHA `89b053e7332e20f284c2de28d2f2365a3a9cf157d017c628481d9715ec5a1a77`와
  원본 capture/runtime/journal/참조를 보존한다. 기존 generator/collector 파일을 직접 수정하지 않았다.
- 새 분석/실험 root는 `~/morai_cap/base5_postcollection_261010`이다. 새 결과를 기존 수집에 import하지 않는다.

## 완료한 전수 읽기 전용 분석

`report_base5_postcollection.py` → `observed_v140.json`.
status의 누적 active는 13.6575시간이다. 부모 import 누적시간들을 더하지 않는다.
하드링크/중첩 root를 중복 집계하지 않은 native timer는 5,515 load interval이다.
resume 후 timer가 이전 segment를 덮어쓰고 interpreter/부모 QA가 빠지는 한계가 있으므로
native timer 합을 전체 wall이라고 부르거나 active와의 차이를 전부 제거 가능한 overhead라고 부르지 않는다.

| native 단계 | timed load당 초 |
|---|---:|
| pose/map 지지 확인 |2.064|
| 서비스/제어 준비 |1.917|
| 파일 write/안정화 |1.159|
| Ego 정지 증거 |0.342|
| provenance/충돌/센서 audit |0.080|

전체 5K의 raw component 11,372개 중 이 분석 당시 320 crop/ROI 조건을 통과한 유효 GT는 10,959개,
제외된 GT는 413개(3.63% GT)다. 풍부한 단서 8,272개(75.48%), LiDAR 1–4점이며 camera 조건 미확인 697개(6.36%),
camera-only 586개(5.35%)다. 이는 장면 실패율이나 모델 정확도가 아니다.
train에 자산 122종이 모두 존재하며 분석용 풍부한 단서가 5개 미만인 자산은
OBJ_Hyundai_Xcient_Mixer(3/3), OBJ_Kia_K5(4/4), OBJ_Kia_Niro(4/4)다.
각 분모는 train 유효 component GT이며 최종 crop 결정 후 다시 계산한다.

범용 regular/dense 유효 primary 1690/보조 4603 parent 배치에서 2점 크기군은
primary 806(47.7%)/보조 2357(51.2%)다. 가시 보조 parent 2191이며
복합 component를 포함한 해당 크기군 유효 GT는 primary 874/보조 2313이다.
2점=작은 화면 객체가 아니고 비용/차종 pool 조건이 함께 작용한다.
현재 점수제 폐지·작은 자산 제외·보조 가중치 변경을 뒷받침할 정도의 편중 근거는 없다.
기존 성공분과 다음 환경의 primary 할당·예산 비율을 유지한다.

## 새 환경 생성 구현

- `base5_spatial_diversity.py`, `prepare_base5_fresh_environment.py`, `prepare_base5_fresh_suite.py`.
- 원래 구성표/primary/거리·방향/미션 phase/split은 보존하고 환경별 geometry seed를 새로 만든다.
  config seed만 바꾸거나 기존 payload를 복사하는 방식이 아니다.
- 범용 route_general은 허용 도로/구역/physical-link split interval에서 연속 arc를 뽑는다.
  실제 final XY의 높이·footprint 검사, NPC 실제 경로, 횡단축, 정지·실제 이동 정책은 기존 것을 사용한다.
- 처음 좁은 strata만 적용한 pilot은 원거리 곡선/전용구역 근처에 unfillable cell을 만들었다.
  거리·방향별 primary 배치 가능성 proxy를 먼저 검사하고 compatible interval에 quota를 배분한다.
  초기 12m local window,64회 후 24m,256회 후 48m로 bounded 확대한다.
  전체 지도 재추첨으로 어려운 quota를 몰래 버리지 않으며 사용 window/arc/final XY를 기록한다.
- 같은 논리 slot이 다른 환경에서 새 geometry가 된다. unique geometry group/token과 물리 link split을
  구분한다. final-support link/인접 100m시야 중복을 별도 감사하며 완전한 지리적 held-out을 주장하지 않는다.
- `run_base5_capture_campaign_spatial.py`와 `run_base5_fresh_batch.py`가 quality 재배치에도 저장된
  공간 plan을 연결한다. 기존 Sunny13 runtime/출력으로 새 adapter를 강제 resume할 수 없다.
- 생성 입력/spec/hash는 새 root에 저장한다. 실제 UI 날씨/시간 변경과 본 수집 승인은 별도다.
  진단용 v141/v141b/v141c 중간생성은 보존하되 최종 다음 환경 입력으로 바로 쓰지 않는다.

## 시간 최적화 실험 구현과 경계

### clean 재측정·장기 회귀 완료

서비스 확인 순서를 유지한 후보의 clean baseline(A2/A3) 평균 389.488초 대비
optimized(F1/F2) 평균 301.647초, **22.553% 단축**이다. 모든 반복 64/64 승인/검토 0이다.
별도 200장 장기 검증은 955.731초/200 승인/검토 0이고 실제 이동 장면 8장도 226.109초/8 승인이다.
200장 및 64장 완료 resume은 ledger/raw 불변·추가 촬영 0이다. 이 값은 같은 payload의 부모 전체 active이며
완료 resume 검증에 걸린 0.4–0.7초를 재촬영 시간으로 사용하지 않는다.
실제 동적 준비는 단축하지 않았고 본 5K는 아직 이 runtime으로 수집하지 않았다.

| 같은 64 payload 반복의 단계 | baseline 초/load | optimized 초/load |
|---|---:|---:|
| pose/map 확인 |1.836|1.506|
| 서비스/제어 준비 |1.920|1.252|
| session/ROS/preflight |0.138|0.035|
| 파일 write/안정화 |1.179|1.173|
| Ego 실제 정지 증거 |0.340|0.341|
| 충돌·센서·provenance audit |0.078|0.075|
| native timer 밖 부모 비용 |0.333|0.080|

실측 단계분해이며 pose 차이 전부를 cache의 인과효과로 단정하지 않는다.
확실한 변경은 조건부 0.7초 setup 대기 조정과 chunk 4→16의 제한된 준비 재사용이다.
GT·파일 안정화·정지 검사를 빼서 줄인 시간이 아니다. 기존 최종 13.6575시간과 이 pilot의 7.224시간
외삽 차이를 전부 22.55% 최적화 효과라고 해석하지 않는다(실패·보충·표본 구성이 다름).
200 정적/8 이동을 4822 정적/154 교차로 이동/24 음영 이동으로 가중한 예상은 7.224시간,
실패/보충/새 환경 여유를 둔 운영예산 8–11시간이다. 확정된 full-5K 소요시간이나 통계적 신뢰구간은 아니다.
pilot 저장가중 27.036GiB/5K, 보수 공간예산 35GiB/5K를 별도로 둔다.

### 25K 생성 중 근접 rider 사례

`fresh_suite_final_v141/sunny15`의 02629는 3–5m 전방 Bicycle_Female1 quota였다.
512개 CPU 후보 중 primary 간격실패 502개, 보조 간격실패 6개, 거리 bin 실패 4개로 성공이 없었다.
Ego 후륜 기준 3–5m에는 차체 앞끝이 포함돼 collision-free 영역이 좁다. 위치 window를 더 넓히거나
자산/근접 quota를 빼는 것이 해결책은 아니었다. 같은 plan/자산/거리/sector/검사를 유지한
attempt 602(603번째 후보)가 0.348m clearance로 통과했다. 따라서 생성전용 유한예산 4096개로 보완하고 후보 수/사유를 기록한다.
이는 **offline JSON 생성** 비용이며 4096번 실제 캡처/retry가 아니다. geometry/GT/QA 기준을 완화하지 않는다.
실패 v141은 보존하고 `fresh_suite_final_v142`에서 코드 동결 후 5 환경 전수 생성을 완료했다.
25,000개 payload/token의 중복은 0이며 원래 primary·거리·방향·미션 phase·split 계약의 변경도 0이다.
각 환경의 4255 train/745 val·정적 4822/이동 178·122종 primary 최소 할당 검사를 통과했다.

| 환경 | unique base XY(3dp) | unique final Ego XY(3dp) | 최대 CPU 후보 수 |512개 초과 slot|
|---|---:|---:|---:|---:|
|sunny11|4992|4999|434|0|
|sunny15|4996|4999|603|1|
|foggy11|4993|4998|563|1|
|foggy13|4992|5000|407|0|
|foggy15|4991|4999|229|0|

매 환경의 범용 slot 1882개에 연속 도로 위치 샘플링을 적용했다. Sunny11 범용 장애물 770개와 사람 계열 830개는
각각 기존 기준 위치 98개에서 770/830개로 늘었고 기존 기준 위치와의 일치는 0이다. 고주로 740/로터리 350 등은 이미
연속 미션·경로 샘플링을 사용하므로 그 규칙을 유지한다. 기준 위치가 다르다고 배경까지 독립적이라고 해석하지 않는다.
physical-anchor-link의 split 충돌은 0이지만 final support link 9개는 train/val 양쪽에 있다(회피 관측 재배치).
인접한 100m 시야의 공간 중복은 남으며, 합성 val을 실전의 독립 rosbag 평가로 대체하지 않는다.

### 추가 후보 중단 / 2026-10-10

첫 baseline 64장 두 회의 평균은 389.556초, static-ready+chunk 16 두 회의 평균은 301.620초로 22.57% 단축됐고,
모두 64/64다. delay-only chunk 4는 350.924초다. 이는 200 scene/이동/새 geometry 승격 전 기록이다.
read-only service 준비를 pose 대기와 겹친 추가 후보는 chunk 4의 64/64 이후 chunk 16 e1에서
`b5v2_03511`의 `service [/Service_MoraiEventCmd] unavailable`로 중단됐다.
early probe가 이전 SceneNetwork 서비스를 확인한 뒤 reload로 사라지는 경쟁 조건으로 추정한다.
scene boundary/final Park는 검증됐으나 unsafe target failure를 부족분으로 자동 건너뛰지 않았다.
실패 원본/32선택/로그/계약을 보존하고 강제 resume하지 않는다.
이 후보는 **탈락**이며 새 fresh runtime의 `--runtime-overlap`도 실행 전에 거부한다.
조건부 0.7초 대기만 조절하되 service 확인 순서를 유지한 기존 후보로 clean 재측정/200장 검증을 완료했다.

### 새 환경 유한 보충·진입점 / 실제 신규 부족분 검증 완료

`run_base5_fresh_completion.py`는 fresh terminal batch를 native read-only로 복원·검증한 뒤
별도 selection을 사용한다. 기존 Sunny13의 하드코딩 driver로 원래 출력을 수정하지 않는다.
패밀리·미션·semantic pool·split·실제 primary gait와 이동 mode별 최소 인원은 유지한다.
자산·거리·방향·개수 변경은 명시적 delta와 adopted_manifest에 남긴다. 희귀 primary의 최소 할당에 미달하면
다른 자산으로 대체하지 않는다. 같은 payload 재촬영·known SHA 재사용·partial/unsafe child 자동 reload를 금지한다.
셀당 누적 3회(최대 5), 실행당 1800초(최대 3600)의 상한을 적용한다. PAUSE/STOP/완료 후 resume의 촬영은 0이다.
강제 3회 review 후 다시 실행해도 상한이 초기화되지 않는 단위 검사를 추가했다.
정적 장면은 검사를 유지한 one-shot 방식이며, 실제 이동 장면은 독립 준비를 유지하는 spatial campaign이다.
`launch_base5_fresh.py`는 검증된 release·코드·manifest 해시와 명시적인 MR UI 환경 확인을 검사한다.
한 환경의 본 수집과 유한 보충을 하나의 진입점으로 묶되, 새 보충 경로의 실제 검증 전에는
`--complete`를 본 수집 시작 전에 거부한다. 이번 실제 검증은 완료했으며 실제 25K는 자동 실행하지 않았다.

- `run_base5_capture_optimized.py`는 frozen collector AST의 명시적 0.7초 setup 대기만 opt-in 조건부로 바꾼다.
  정적/빈배경+fresh post-load transport일 때만 생략, 동적은 원래 대기를 유지한다.
  fresh service proxy/검증 응답/한정 재시도·매 scene network reload·Park·실제 정지·접촉·0.5초 파일 안정화는 그대로다.
- source AST 구조가 달라지면 fail-closed. adapter/source hash를 session runner_options에 기록한다.
- `run_base5_batch_optimized.py`의 별도 guard는 code/manifest/chunk/profile 변경을 거부한다.
  profile별 새 output만 사용한다. 최대 16 scene chunk의 ROS/map 공유이지 chunk를 넘는 영구상주 worker라고 부르지 않는다.
- 사전에 정한 정적 64장/이동 8장 payload 목록은 `runtime_benchmark/registration.json`이다.
  결과/score를 보고 장면을 고르지 않았다. baseline chunk 4 → delay-only chunk 4 → combined chunk 16
  → combined chunk 16 → baseline chunk 4의 반복/역순 비교를 완료했다.
- CPU 생성이 초기 baseline/첫 delay 일부와 겹쳤으므로 나중의 clean 역순 pair와 일치하는지 확인한다.
  20% 이상 full parent 비용 감소와 승인/안전 유지 전에는 자동 승격하지 않는다.
  64 scene 합격 후 별도 200 scene 장기/8 이동 회귀와 resume 검사도 모두 통과했다.

## 새 배치 실제 MR·보충·변환 최종 결과

- 새 정적 64장: 최초 63 승인/1 부족, 부모 활성 시간 421.379초(chunk 4, pause/resume 및 native quality 재배치 포함).
  01521은 native quality 재배치로 복구됐고, 04635는 얇은 Lane_Separation_Bar와 혼합 보조 객체의 가시성이 부족했다.
  `fresh_geometry_completion_v3`의 첫 새 후보로 04635도 복구해 64/64를 확보했다. 원래 63행·부모/실패 raw·journal은 보존한다.
- 새 이동 8장: 최초 7 승인/1 부족. 음영 Dog1의 정적 지면 준비가 검사에 걸렸고,
  `fresh_geometry_animated_completion_v3`의 첫 새 후보가 독립 지면 준비와 실제 walking 증거 검사를 통과해 8/8을 확보했다.
  원래 primary gait·이동 mode별 최소 인원과 source split을 모두 검사했다. 지면 검사 완화나 정적 대체가 아니다.
- 정적·이동 유한 보충은 각각 1시도였으며 부모 해시는 불변이다. 종료 후 resume에서도 journal·선택·시간·raw가 불변이다.
  native batch의 selected row에 `scenario` 필드가 없는 경우의 adopted_manifest 직렬화 오류와
  background의 distance-bin 부재를 실제 roundtrip에서 발견해 수정했다. 실제 보존 scenario SHA를 검증하며
  초기 manifest payload로 잘못 대체하지 않는다. 같은 자산이라도 다른 split의 donor가 source fallback을 막지 않게 했다.
  실패 preflight v1/v2는 보존했고 수정 코드로 강제 resume하지 않았다(촬영 미실행).
- CPU infos 64장과 기존 Sunny13 infos 5K의 호환성 병합은 5064장이며 중복·group split 누수는 0이다.
  검증용 pilot은 본학습·30K에 포함하지 않는다. 실제 새 날씨 조건을 검증한 것도 아니다.
- `collector_release_v142/release.json`에 최종 helper 절대 경로 SHA·입력 suite SHA·profile·유한 보충·실측 근거를 동결했다.
  5개 환경의 `launch_base5_fresh.py --prepare-only`가 모두 통과했으며 child와 실제 본 수집은 실행하지 않았다.
- `base5_fresh_dashboard.py`는 같은 source·부모 계보의 `_completed` 선택을 자동으로 관측한다.
  부모와 보충 성공분을 중복 합산하지 않으며 다른 환경의 보충도 suite에 반영한다. 큰 5K state는 8KiB 계보 header만 읽는다.
  원래 8075 관측기·수집 runtime은 바꾸지 않았다. 새 8082는 전체 30K 중 Sunny13 5K만 승인된 상태다.
- 최종 Sunny13 ledger SHA는 그대로다. MR worker 종료와 Park를 확인했으며 production·GT·threshold·model·GPU 작업은 변경하지 않았다.
- 최종 `test_base5*.py` 전체 단위 검사: 554개 PASS, 73.771초. `git diff --check`도 통과했다.

## 다음 실행

- 5개 환경 25K의 offline manifest·coverage·token·split·위치 검사는 v142에서 완료했다.
- 수집기/새 배치 live·200 scene·실제 이동·보충·resume·최종 Park·CPU 변환은 완료했다.
- [새 단일 진입점 런북](261010_base5_fresh_environment_runbook.md)에 따라 UI 환경을 확인한 후 새 5K씩 수집한다.
  완료 후 resume은 상한을 초기화하지 않으며 native QA·안전 실패를 무시하지 않는다. 앞으로의 모든 부족분 복구를 보장하지 않는다.
- 이 수집기 작업을 마칠 당시 다른 환경의 실제 25K와 GPU·crop·BEVPool·cache·초기화 실험은 미시작이었다.
  이후 GPU 실험 진행 상태는 [입력·pooling 최적화 근거](../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md)를 따른다.
  GPU 작업 전 MR을 꺼야 한다. crop·투영 확정 후 visibility/infos를 다시 구성하고 동일 조건 4090 비용을 검증한 뒤 본학습한다.
