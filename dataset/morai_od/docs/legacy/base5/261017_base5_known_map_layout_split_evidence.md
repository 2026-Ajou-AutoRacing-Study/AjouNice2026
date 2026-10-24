# 알려진 대회 지도 학습용 배치 그룹 분할 근거

2026-10-17 KST. 코드·CPU 검증 근거다. 본수집·재학습·실주행 성능 승인을 뜻하지 않는다.
기존 Sunny13 5K, native 원본, selected ledger, v155m 및 과거 release는 수정하지 않았다.
아래 v156b 수치는 초기 분할 snapshot이며, v156e의 후속 CPU 검사·전체 회귀 이력과 구분한다.
공통 참조 문제 수정 뒤 전체 회귀 v3와 사전 등록 v3까지 완료했다. 정식 paired는 120장 계획 중
26장·13쌍에서 사전 futility 한계로 중단했고, 확대 실험은 종료한다. 신규 bulk 승인은 없다.

## 확대 실험 종료와 다음 적용 범위

정식 paired의 최초 관측은 원형과 확대 각 13장이다. 예비 2장 및 별도 warm/cold 진단을 이 분모에
합산하지 않는다. 실제 결과는 다음과 같다.

| 조건 | 최초 관측 | 승인 | review | WoodBox native 크기 mismatch 장면 |
|---|---:|---:|---:|---:|
| 원형 | 13 | 5 | 8 | 5 |
| 확대 | 13 | 6 | 7 | 5 |

각 조건 60장 중 최초 승인 54장 이상이라는 사전 기준에서 비승인 7장 도달은 더 이상 통과할 수
없는 한계다. 이 futility 기준으로 STOP했고 최종 보존 집계는 26장·13쌍이다. 전체 120장을
완료하거나 통과한 것이 아니다. 원형에서도 같은 WoodBox mismatch가 발생했으므로 확대만을
유일한 원인으로 단정하지 않는다. 다만 이번 확대 수집 실험은 여기서 종료한다.

실제 근거는 `artifacts/evaluation/base5_scale_qualification_261017/capture_stability_final_v3/part_a`다.
상태는 graceful STOP에 따른 `interrupted`이며 종료 `park_verified=true`를 확인했다.
`ledger.jsonl` SHA는 `5b3b4daf8986ff0d6ab92d65cfdeabcf41d5f41fcbed1c2d088acde8f9ed8133`,
`shutdown_safety.json` SHA는 `1d76dfb132c214179748334192731c2cdf5643817e41f772df5687e25c51c8a8`이다.
part_b, 최종 smoke 24장, 그 smoke의 최종 loader 검증과 새 release는 실행하지 않았다.
`completion_live_qualified=false`를 유지하며 본수집·자동 보충·신규 bulk 승인은 없다.

분할·계보 고정·geometry leakage 감사 개선은 유지한다. 다음 scale 1 source가 정해지면 그 실제
scenario/metadata를 대상으로 overlay를 새로 생성하고 train 전역 coverage·희귀 자산·중복 누수를
다시 감사한 뒤 소량 native 및 입력 검증을 해야 한다. 원형 조건에도 review가 있었으므로
scale 1 후보를 자동 승인하지 않는다. 아래 v156b/v156e의 4,250/750 분할, 69 link·316개 셀,
등록 profile 수나 경로 거리 수치를 원형 fallback의 최종 결과로 전용하지 않는다.

## 결정과 구현

새 정책은 `known-map-layout-group-v156`이다. 동일한 대회 지도의 배경을 train과 val이 공유하는 것은
의도한 조건이다. 전체 도로 link를 한 split에 독점시키는 대신, 실제 반복 배치와 명시적인 시간연속
장면 그룹을 분리 단위로 삼는다. 서로 다른 환경에서 독립 생성된 같은 논리 quota slot은 그룹으로
묶지 않는다. 같은 geometry의 날씨 변형은 같은 그룹이다.

`base5_layout_split.py`는 final Ego XYZ/yaw와 actor의 DataID·container·XYZ/yaw·size/scale·motion/path를
정규화한다. 완전히 같은 배치 및 위치 0.05m·yaw 0.25도 이내의 보수적인 근접 반복 배치를 묶는다.
이 기준은 지도 시야의 독립성이나 실제 native 동작 중의 모든 모양을 인증하지 않는다.
이웃 도로의 100m 시야 중첩은 허용되는 알려진 지도 조건이며, 무누수 인증과 혼동하지 않는다.

분할의 우선순위는 다음과 같다.

1. 실제 배치·시간연속 그룹의 train/val 혼합 금지.
2. 관측된 final Ego link·10m XY 셀 및 미션 구간별 train 사례 보존.
3. asset과 등록 scale profile별 train 독립 그룹 최소 5개 보호. 전체가 5개 미만이면 전부 train이다.
4. family·거리 계약·방향·미션 phase·yaw·topology를 계층화해 약 85/15를 맞춘다.

85/15는 hard gate가 아니다. 희귀 자산·공간 coverage를 지키며 val을 채우지 못하면 부족 수를 보고한다.
위 최소 개수는 학습 충분성이나 정확도 향상을 보장하지 않는다.

## 초기 snapshot: v155m 배치를 보존한 v156b 5,000장 CPU 결과

입력은 `artifacts/evaluation/base5_mission_scale_261016/registered_scale_v155m/manifest.json`이며 SHA256은
`962854897cca976290169564549c173542fe78f624f5bf1d566706005b1ffff4`다.

당시 출력은 `artifacts/evaluation/base5_layout_split_261017/candidate_v156b`다.

| 항목 | 원래 capture split | 새 학습 split |
|---|---:|---:|
| Train / val | 4,255 / 745 | 4,250 / 750 |
| 정적 미션 before | 기존 link 계약 | 255 / 45 |
| 정적 미션 inside | 0 / 120 | 102 / 18 |
| 정적 미션 after | 기존 link 계약 | 153 / 27 |
| 로터리 | 350 / 0 | 298 / 52 |
| 관측 final Ego link 중 train 포함 | 62 / 69 | 69 / 69 |
| 관측 final Ego 10m 셀 중 train 포함 | 293 / 316 | 316 / 316 |

독립 배치 그룹은 5,000개이며 exact/근접 geometry의 cross-split 그룹은 0개다.
122종 asset의 train 누락은 없고, 최소 train 독립 그룹은 5개다.
원형을 포함한 등록 profile ID 21종의 최소 train 독립 그룹은 9개다.
이는 planned parent 집계다. 최종 native 가시 parent·component GT 집계와 다르다.

지면/구역 screening을 통과한 허용 경로를 약 10m 간격으로 244점 점검했다.
Train final Ego와의 최근접 거리는 243/244점에서 10m 이내, 244/244점에서 25m 이내이고 최댓값은
12.355m다. Val은 각각 228/244점·237/244점이며 최댓값은 54.393m다.
이것은 경로 표본과 final Ego의 거리이지 모든 도로 면적·센서 가시성의 인증이 아니다.

산출물 SHA256은 다음과 같다.

- `candidate_v156b/manifest.json`: `b6c3117a9173340b37d382fa1e44c984613f21590e69de23e02d26bbea302e43`
- `candidate_v156b/split_overlay.json`: `37055426c3ae847bf60362ffd37c4b682d8fa26d8e298e1a5dfd447123c4721b`

v156b는 생성 당시 도우미 hash를 가진 진단 snapshot이다. 이후 복구 합본 finalizer를 추가했으므로
이를 최신 source/runtime 동결 release로 재사용하지 않는다. 확대 실험 종료 뒤 다음 scale 1
source에서도 기존 산출물을 복사하지 않고 새 overlay·CPU 감사·소량 검증과 별도 동결이 필요하다.

10/17 후속 통합에서는 저장 음영 지면 준비를 먼저 materialize한 뒤 원형 QA와 분할을 적용했다.
확대 실험에 사용한 큰 규격 후보는 `base5_scale_qualification_261017/candidate_large_v156e`이며 위
v156b와 별도 snapshot이다. 그 후보에서 69 link·316개 10m 셀의 train coverage와 4,250/750
분할을 다시 확인했다. 다음 scale 1 source의 분할 수치나 승인을 뜻하지 않는다.
최신 전체 경로·SHA·실제 입력·native 승인 상태는 [통합 검증](261017_base5_scale_split_qualification.md)을
따른다. 아래 초기 구현 검증 이력의 테스트 수는 당시 기록이며, 후속 전체 회귀 결과는 뒤에서 별도로 기록한다.

## Capture split과 파생 학습 split의 분리

`prepare_base5_layout_split.py`는 기존 scenario를 바꾸지 않는다. 새 metadata 복사본에는
`training_split_overlay` 참조와 원본 metadata path/SHA만 추가한다. 기존 `split`, frozen spec,
actor population, 거리·동작·scale 계획은 유지한다. Manifest의 `derived_training_split`과
`derived_layout_group_id`는 학습에 쓰는 값이며 기존 capture split을 덮어쓰지 않는다.

따라서 frozen scale plan과 수집기의 기존 계약을 바꾸지 않고 inside train·로터리 val을 만들 수 있다.
실제 5,000개 entry에서 collector의 scale `validate_entry`와 frozen scale `validate_plan`이 통과했다.
정책을 바꾼 새 생성물을 기존 output에 강제 resume하지 않는다.

Infos 변환과 입력 qualification은 태그를 자동 발견하거나 `--split-overlay`를 명시적으로 받는다.
원래 split은 `source_split`, 파생 split은 파일 분할에 기록한다. Geometry 그룹은 infos의
`scene_group_id`로 전달하므로 기존 환경 merge의 같은 그룹 무누수 검사는 유지된다.
Overlay checksum·payload/geometry·원래 split·source metadata pin이 다르면 중단한다.

Launcher/collector의 사전 검사에서는 전체 source hash를 한 번 감사하고, entry별 binding을 검사한다.
소량 smoke manifest는 `allow_subset=True`로 전체 승인 overlay에 대한 부분집합 binding만 확인한다.
부분집합을 독립적인 5K coverage 승인으로 해석하지 않는다.

## 새 geometry 생성의 선택적 변경

`config.layout_split_policy`를 명시한 새 후보에서만 whole-link 제약을 해제한다.
범용 연속 Ego 샘플링은 모든 허용 link의 interval을 사용하며, 미션 extent도 before/inside/after
각각을 두 split에서 사용할 수 있다. 구역·지면·배치·가시성·충돌 제약은 완화하지 않는다.
기존 flag/config가 없는 생성과 frozen resume 경로는 기존 계약을 유지한다.

`prepare_base5_fresh_environment.py`와 `prepare_base5_static_mission_extent.py`의
`--known-map-layout-split`은 이 새 후보 경로를 선택한다. 새 geometry의 초기 split은 잠정값이며,
전체 생성 뒤 coverage 제약을 가진 overlay 할당이 필요하다.
후처리 re-split은 기존 장면의 학습 소속만 바꾸므로 새로운 Ego 위치나 장면을 생성하지 않는다.

## 복구 proposal과 최종 합본

원본 overlay의 scenario/geometry binding은 새 복구 geometry를 거부하는 것이 정상이다.
`register_recovery_overlay`는 명시적인 부모 ID·원본 scenario SHA·ordinal을 확인하고 새로운
proposal용 overlay를 만든다. 부모의 effective split/group, actor identity/motion/scale roster,
거리·phase·yaw 계약 및 train coverage floor를 지킨다. 부모 metadata의 비기하 필드와 원본 source
hash, 기존 recovery binding도 검증한다. Alias 충돌이나 다른 실제 배치와의 중복은 중단한다.

서로 다른 proposal은 독립적으로 원본에서 생성되므로, 각각의 검사를 통과해도 성공분끼리 중복될 수
있다. `finalize_base5_layout_split.py`는 승인된 전체 selected ledger와 adopted manifest를 입력으로
실제 선택된 모든 geometry를 다시 감사한다. 서로 다른 split에서 새로 겹친 그룹, train floor 손실,
중복 quota/runtime ID/raw root, 끊긴 부모 계보, source/raw/audit 변조를 허용하지 않는다.

합본은 native raw metadata와 원본 selected ledger를 고치지 않는다. 새 sidecar metadata·manifest·
합본 overlay·`derived_selected_ledger.jsonl`을 만든다. 파생 ledger의 태그를 읽으면 raw에 남아 있는
서로 다른 부모/proposal overlay를 합본의 검증된 ancestry로 연결해 infos를 생성할 수 있다.
새 합본 overlay를 명시적으로 주어도 같은 검증을 수행한다.

새 sidecar는 검증된 계획 metadata의 core를 보존한다. 수집기가 RAW에 덧붙인
`capture_runtime_*` 같은 진단 필드는 계획 core에 합치지 않고, 원본 RAW metadata/audit의
별도 SHA와 계보로 고정한다. 따라서 materialized shadow의 준비된 actor/path/spec 계약도
그대로 유지하면서 실제 runtime 증거를 보존한다. RAW 진단 필드 변경 역시 최종 계보 검사에서
거부한다. 등록된 shadow를 materialize한 fixture에 runtime 필드를 추가한 뒤 실제 shadow
검사까지 통과하고, 이후 RAW 변조를 거부하는 회귀 검사를 추가했다.

`run_base5_fresh_completion.py`는 부족분이 0이 된 시점에 finalizer를 연결한다. 완료 캐시는 selected
ledger SHA로 구분하며, 완료 resume는 산출물·raw/source pin을 다시 검사하고 다시 쓰지 않는다.
복구/합본 도우미 hash는 completion resume 계약에 포함된다.

원형 native geometry review 참조가 있는 경우 복구 전에 원본 metadata의 참조를 새 metadata로
명시 복사하고 `load`로 검증한다. 최종 합본도 config와 같은 참조인지 확인하며, 기존 계획 필드의
보존 검사로 참조 삭제·변경을 거부한다. 원형 표를 GT로 복사하거나 native annotation을 수정하지 않는다.

## 초기 구현 검증 이력과 후속 상태

다음 항목은 초기 분할·합본 구현 단계의 부분 검사 기록이다. 현재 전체 회귀의 합계나 새 native
승인 수로 읽지 않는다.

- 분할·합본 테스트 18개 통과: 연속 전역 interval, inside train, 실제/근접 geometry 그룹, 희귀 floor,
  독립 복구 proposal 간 cross-split 중복, alias/source 변조, unit 참조, 완료 캐시 검증을 포함한다.
- 기존 extent 테스트 14개, 환경 merge/변환 테스트 7개 통과.
- Fresh completion 테스트 16개 통과.
- 입력 qualification 테스트 16개 통과. 합본 ancestry를 사용해 raw train 장면을 effective val로
  Q90 JPEG/upright/3-class infos까지 변환하고 원본 byte 보존을 확인했다.
- 전체 source의 scale entry/frozen plan 검사 5,000/5,000 통과.
- `py_compile` 및 `git diff --check` 통과.

초기 snapshot 당시 남은 항목 중 overlay 재생성과 후속 full5K CPU 검사는 v156e까지 진행했다.
이후 정식 paired는 26장·13쌍에서 중단했다. 남은 확대 실험의 part_b·최종 smoke·loader·release를
추가 실행해 완료할 계획으로 읽지 않는다. 다음 scale 1 source의 별도 검증과 이력을 구분한다.
새 복구 경로의 native/live qualification이 별도로
통과하기 전에는 `completion_live_qualified=false`를 유지한다. CPU proposal/합본 검사를 live
복구 승인으로 해석하지 않는다. AP 비교와 미사용 독립 rosbag 검증도 별도 gate다.

분할·합본 구현 기록의 주요 도우미 SHA256은 다음과 같다. 이 목록 자체가 전체 runtime을
동결하거나 새 release를 승인하는 것은 아니다.

- `base5_layout_split.py`: `26cc93acb156fd286a0947bd181e05026a845a2405538740e00a997ae0a4751f`
- `finalize_base5_layout_split.py`: `56bee6211d5ebdd3f608852460dd3fb7d28e95f4d775989824d95e61d8748afc`
- `prepare_base5_layout_split.py`: `40c825fd35d7a56a0300f9f42aaa42fd6b1a0d8011afd8ca7f82c40745160169`
- `run_base5_fresh_completion.py`: `a4d9d0f1acaa302ebb6b7f3b0657039b8905aea951a3310bfbb534876e40d9a0`

일반 commit/push와 CURRENT·README·최종 통합 문서 갱신은 상위 통합 작업에서 수행한다.

## 종료한 확대 실험용 유한 smoke와 paired gate의 구현 기록

기존 v155의 7에셋·14프로파일·17장 도우미와 receipt는 바꾸지 않았다.
새 `prepare_base5_final_bounded_smoke.py`와 `finalize_base5_final_bounded_release.py`는 현재
qualified 프로파일 집합을 읽되 source·정책·도우미·수집/변환/loader 코드와 입력 파일을 엄격히
고정한다. MMDet의 로컬 config 상속과 dataset/transform 코드도 runtime 고정에 포함한다.
아래는 이번 확대 실험에 사전 고정했던 계약이다. 구현을 보존하되 확대 수집을 재개하라는
실행 지침이나 다음 scale 1 후보에 그대로 적용할 승인 계약으로 해석하지 않는다.

v156e 후보를 읽기 전용으로 검사했을 때 full5K의 실제 scale/unit/layout/shadow 계약이 통과했고,
24장으로 17프로파일·미션 전/중/후·4동작·effective train/val·존재하는 모든 확대 context/역할을
포함할 수 있었다. 보호 control은 materialized moving shadow 2장, 실제 움직이는 두 횡단보도
각 1장, 원형 원거리 차량 4장이다. CargoBox iso300의 실제 재고 3장도 모두 포함한다.
이는 당시 선정 가능성 검사이며 등록이나 새 native 승인이 아니다. 이후 진단 및 사전 등록 시도의
상태는 [통합 검증](261017_base5_scale_split_qualification.md)을 따른다. 아래 gate의 구현·부분 검사와
정식 paired 120장 및 최종 release의 실제 통과를 구분한다. 실제로는 위 26장 중단 후 최종
smoke를 실행하지 않았다.

당시 사전 등록과 검증에 사용한 순서는 다음과 같다.

1. 최종 full5K에서 `prepare_base5_scale_stability.py --pairs 60 --minimum-per-profile 3`으로
   60쌍의 원형/확대 진단을 생성한다. 새 smoke 등록에는 이 manifest가 필수다.
2. smoke 도우미가 paired 생성기를 임시 경로에서 재실행해 source 계보, 모든 scenario와 metadata,
   actor UID, 순서와 config를 대조한다. 실제 source layout 60개, 프로파일별 최소 3개,
   CargoBox 3배 재고 전수, AB 30:30을 확인한다. 임의 UID 변환이나 소급 재해석은 허용하지 않는다.
3. 새 smoke 등록은 기본 24장, 절대 상한 64장이다. source와 metadata는 그대로 쓰며,
   별도 진단이나 과거 성공 장면을 smoke 승인으로 세지 않는다. paired와 smoke의 실제 native
   event는 이 등록 시점 이후여야 한다.
4. paired gate는 전용 capture root 안의 모든 session과 실제 raw ledger를 빠짐없이 받는다.
   실제 사용 manifest가 등록 deck 또는 정확한 subset인지 확인하고, 현재 `prepare_session`을
   임시 경로에서 재현해 전체 계약을 비교한다. helper, sensor, runner, 최종 Park와 필요한
   between-scene Park를 검사한다. orphan session/ledger, 빠진 최초 관측, 순서 변경은 거부한다.
5. 실제 RAW source·unit/scale QA와 승인 증거를 대조한 뒤 원본 ledger로 stability report를
   다시 생성한다. 최초 120개, 각 조건 60개와 독립 source geometry 60개를 검증한다.
   각 조건 최초 승인율 90% 이상, 확대의 최초 비승인율이 원형보다 5%p 초과하지 않을 것,
   승인 장면당 ledger 비용이 원형의 110%를 초과하지 않을 것을 사전 고정한다.
   프로파일별 실패율·비용 차이도 같은 비교 기준으로 검토한다. 원인 미상·인프라·안전 원인을
   구분하고, `failed` 또는 `ordinary_deficit=false`는 비율과 별개로 보류한다.
6. 최종 source의 실제 smoke RAW에 대해 native QA, 충돌 없음, Park, 부분 4장 보존/resume,
   완료 후 무촬영 resume, Q90/upright/3-class 입력과 실제 MMDet CPU train/val loader를 검증한다.
   loader의 token 집합·원 split·frame별 GT를 실제 package infos와 대조하며, tensor는
   `4×3×512×1056`, `torch.uint8`이어야 한다.

비용은 재시도까지 포함한 관측 ledger duration을 최종 승인 장면 수로 나눈 값이다.
프로세스 바깥의 시작·운영자 대기·전체 wall-clock 비용은 포함하지 않는다. 목적형 프로파일 표본의
조건부 결과를 full5K 생산 실패율이나 AP로 일반화하지 않는다.

전체 CPU 회귀는 다음 모드로 실행한다. 임의 `-k`나 사용자 exclude는 받지 않는다.
pytest 환경의 숨은 addopts/plugins와 외부 config를 차단하고, source/test 해시의 전후 일치와
실제 JUnit SHA를 하나의 receipt에 연결한다. host에 torch가 없는 기존 Base3 segmentation
metrics node 1개만 정확히 고정 제외하며, 나머지 static Base3 검사는 포함한다.
첫 실제 전체 실행은 1,816개 통과 후 이 legacy node가 제외되지 않아 실패했다. 절대경로
`--deselect`가 pytest의 상대 nodeid와 맞지 않는 문제를 상대 nodeid로 수정했다. 실패한 v1
XML과 산출물은 보존하며, 상대 nodeid의 실제 제외를 작은 독립 pytest subprocess로 검증하는
회귀 검사를 추가했다. 수정 뒤 신규 두 검사 파일의 119개가 통과했다. v1을 성공 receipt로
승격하지 않으며, 아래 v2 결과는 새 출력에서 실제로 다시 실행한 결과다.

| 전체 회귀 snapshot | 실제 결과 | 적용 범위 |
|---|---|---|
| `regression_final_bounded_v1` | 1,816 passed, legacy node 1개 실패 | 제외 옵션 오류의 실패 근거로 보존. 성공 receipt가 아님 |
| `regression_final_bounded_v2` | 1,817 passed, 고정 legacy node 1개 deselected, 132.23초 | 상대 nodeid 수정 뒤 당시 source/runtime의 전체 CPU 결과 |
| `regression_final_bounded_v3` | 1,819 passed, 고정 legacy node 1개 deselected, 136.43초 | 공통 참조 alias 수정 뒤 실제 전체 CPU 결과. native/bulk 승인은 아님 |

v2의 `receipt.json` SHA는 `5ca24a813ffea00f2ce78e2ad0569eaa6d001d3a16073e259b2c4235f7a2bbe2`,
`tests.xml` SHA는 `2dfce70df9348cd68dafeef27b020e5c32d013741c7a4ff5942e6ea694a912c4`다.
XML은 실행된 1,817개에 대해 실패·오류·skipped가 모두 0이며, deselected 1개는 실행 합계와 다르다.

최종 v3의 `receipt.json` SHA는 `45c7c78e8da735beb2a64d4e72a24933c7bcedac643d3e7b1ffa38f2b73c7526`,
`tests.xml` SHA는 `00a2ef8cae6474b0ca224b38e93056bb3008c925a350809339bb9b6abc9a328f`다.
XML은 실행된 1,819개에 대해 실패·오류·skipped가 모두 0이다.

이후 paired 준비에서 새 common-reference gate가 `merge_references`의 `sources`와 `pins`가 같은
객체를 공유하는 문제를 검출했다. 최소 수정 뒤 `stability_large_v156e_v2/manifest.json`을 새로
생성하고 전체 회귀 v3와 `final_smoke_registration_v3/registration.json`을 별도 경로에서 완료했다.
그 뒤 정식 paired를 새로 시작했으나 위 26장에서 중단했다. 사전 등록 문서의 smoke 24장은
선정된 계획이지 실제 취득 수가 아니다. v3 CPU 성공이나 사전 등록을 native/bulk 승인으로
승격하지 않는다.

다음은 향후 별도 검증에 사용할 회귀 명령 템플릿이다. 이번 확대 실험의 v1·v2·v3 경로는 모두
보존 이력이며 재사용 대상이 아니다. `_NEXT`를 새 출력 경로로 바꾸고 기존 경로를 덮어쓰지 않는다.
지금 이 문서 갱신에서 회귀나 촬영을 추가 실행한 것은 아니다.

```bash
python3 dataset/morai_od/scripts/prepare_base5_final_bounded_smoke.py \
  --cpu-regression-only \
  --output artifacts/evaluation/base5_scale_qualification_261017/regression_final_bounded_NEXT
```

최종 release는 그때의 source/runtime에 대응하는 새 CPU 실행 receipt와 모든 실제 gate가
통과해야만 `bulk_ready=true`가 된다.
준비 도우미는 촬영·bulk·학습을 실행하지 않으며, release에서도 `completion_live_qualified=false`를
항상 유지한다. 실시간 자동 부족분 보충 승인은 별도다. 현재 이 release를 발급하지 않았다.

도우미 작성 중의 신규 118개·관련 회귀 245개와 상대 nodeid 수정 후 신규 119개는 단계별 부분
검사 이력이다. 전체 CPU 결과는 위 v2·v3 snapshot과 구분한다. 합성 fixture loader receipt를 실제
tensor loader 승인으로 간주하지 않는다. 별도 과거 입력 검증으로 기존 historical17을 로컬 고정 이미지
`sha256:13488ccac149f41b070e618db5db91bfa3bf2d3d1a6de3dac80aa2e2c948ec46`에서 실제 CPU
loader로 검증했다. workspace는 읽기 전용, 새 결과 폴더만 쓰기 허용하고 GPU·네트워크는
미노출 상태였다. 실제 train/val 각 17장과 GT 43개가 통과했고, 결과는
`artifacts/evaluation/base5_scale_qualification_261017/historical17_cpu_loader_v2/report.json`
(SHA `eda057333dae189d0ffa2fbb2cdb54a6a77dc3cce32f3c778d2b3f7e26a546a2`)이다.
첫 실행의 repository Python 경로 누락은 명령의 `PYTHONPATH`를 명시해 해결했으며 코드를 바꾸지 않았다.
이 과거 17장 결과는 최종 24장 smoke의 실제 loader gate를 대신하지 않는다.

다음은 향후 별도 source의 native 입력 보고서가 준비된 뒤 참고할 CPU 명령 템플릿이다.
종료한 확대 실험의 최종 smoke/loader가 실행됐다는 뜻이 아니다. `_NEXT`는 아직 실행하지 않은
예시 자리 표시자이며, 위 historical17의 v1·v2 실행 이력과 다르다. `QUALIFICATION`과
`OUTPUT_DIR`을 실제 새 보고서와 비어 있는 새 결과 경로로 바꾼다. 전체 workspace를 쓰기 허용으로
마운트하지 않으며, `--gpus`나 ROS/MR 연결은 추가하지 않는다.

```bash
ROOT=/home/luv/AjouNice2026
QUALIFICATION="$ROOT/artifacts/evaluation/base5_scale_qualification_261017/final_smoke_inputs_NEXT/report.json"
OUTPUT_DIR="$ROOT/artifacts/evaluation/base5_scale_qualification_261017/final_smoke_cpu_loader_NEXT"
mkdir "$OUTPUT_DIR"
docker run --rm --runtime=runc --network=none --cpus=2 --memory=8g --shm-size=1g \
  --user 1000:1000 -e CUDA_VISIBLE_DEVICES= -e NVIDIA_VISIBLE_DEVICES=void \
  -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONPATH="$ROOT:/opt/mmdetection3d" \
  -e MPLCONFIGDIR=/tmp/matplotlib \
  --mount "type=bind,src=$ROOT,dst=$ROOT,readonly" \
  --mount "type=bind,src=$OUTPUT_DIR,dst=$OUTPUT_DIR" \
  --workdir "$ROOT/perception/detection3d/bevfusion" \
  sha256:13488ccac149f41b070e618db5db91bfa3bf2d3d1a6de3dac80aa2e2c948ec46 \
  python "$ROOT/dataset/morai_od/scripts/qualify_base5_training_loader_cpu.py" \
  --qualification "$QUALIFICATION" \
  --config "$ROOT/perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_next_training.py" \
  --output "$OUTPUT_DIR/report.json"
```

회귀 v3에서도 같은 bounded 도우미 자체 SHA는 다음과 같다. 공통 참조 수정까지 포함한 전체
runtime의 CPU 고정 근거는 v3 receipt에 있다. 이 hash 목록을 새 scale 1 source의 release로
대체 사용하지 않는다.

- `prepare_base5_final_bounded_smoke.py`: `8630c6fcf8044cdd24cacefd66bb7b07cdf8265203a6ba0067d9d2d1c0308c5b`
- `finalize_base5_final_bounded_release.py`: `13a3fdd85671f1e8224c0b23fdef55cef2a32e1f7a9b11293819e1c07b113d97`
