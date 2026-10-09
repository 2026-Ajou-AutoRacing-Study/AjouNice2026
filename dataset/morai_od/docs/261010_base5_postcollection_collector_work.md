# Base5 종료 후 수집기·새 환경 배치 작업

2026-10-10 / 작업 중 EVIDENCE. 최종 Sunny13 5K와 새 실험/생성 경로를 구분한다.
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

- `run_base5_capture_optimized.py`는 frozen collector AST의 명시적0.7초 setup 대기만 opt-in 조건부로 바꾼다.
  정적/빈배경+fresh post-load transport일 때만 생략, 동적은 원래 대기를 유지한다.
  fresh service proxy/검증응답/한정 재시도·매scene network reload·Park·실제정지·접촉·0.5초 파일안정화는 그대로다.
- source AST 구조가 달라지면 fail-closed. adapter/source hash를 session runner_options에 기록한다.
- `run_base5_batch_optimized.py`의 별도guard는 code/manifest/chunk/profile 변경을 거부한다.
  profile별 새output만 사용한다. 최대16scene chunk의 ROS/map 공유이지 chunk를 넘는 영구상주worker라고 부르지 않는다.
- 사전정한64정적/8이동 payload 목록은 `runtime_benchmark/registration.json`이다.
  결과/score를 보고 장면을 고르지 않았다. baseline chunk4 → delay-only chunk4 → combined chunk16
  → combined chunk16 → baseline chunk4의 반복/역순 비교를 수행 중이다.
- CPU 생성이 초기 baseline/첫delay 일부와 겹쳤으므로 나중의 clean 역순pair와 일치하는지 확인한다.
  20% 이상 full parent 비용 감소와 승인/안전 유지 전에는 자동승격하지 않는다.
  64scene 합격 후 별도200scene 장기/8이동 회귀와 resume 검사도 필요하다.

## 남은 확인

- 최종code 동결 후5환경25K offline manifest/coverage/token/split/위치 감사.
- 새 배치 대표 live, quality 재배치/PAUSE/resume/완료resume 검증.
- runtime 비용 A/B,200scene·이동 회귀, 최종Park/누수/파일완전성 확인.
- finite 부족분 처리의 환경 일반화: 같은 실패payload 무한 반복 대신 bounded 재배치 후
  부족을 명시하고, 사용자 승인한 family/mission/motion/split 보존 대체를 사용한다.
  과거 Sunny13 전용flex driver의 hardcoded 경로를 새환경에 그대로 적용하지 않는다.
- 성공근거에 맞춰 새 런북/suite/dashboard와 예상시간·저장공간을 갱신한다.
  다른25K 실제capture와 GPU/crop 실험은 아직 시작하지 않았다.
