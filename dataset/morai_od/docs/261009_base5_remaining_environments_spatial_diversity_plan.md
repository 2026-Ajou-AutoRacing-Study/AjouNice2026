# Base5 Sunny13 완료 후 나머지 환경의 공간 다양성 계획

2026-10-09 KST / 사용자 합의는 CURRENT DECISION, 생성기 변경과 검증은 PLAN이다.
현재 Sunny13 5K는 그대로 완료·보존한다. 이후 구성표와 주요 자산 할당을 유지하면서
Ego 위치 다양화를 구현·검증하고, 나머지 5환경에서 각각 새 배치로 5K를 수집한다.
총30K를 확보한 뒤 Sunny13 재수집 필요성을 판단한다. 현재 수집 중단·코드 변경·새 생성은 하지 않는다.

## 확정한 순서와 변경 경계

1. Sunny13 본5K를 완료한다. 승인 수·잔여 부족분·최종 Park·collector 종료를 확인한다.
   부족분은 기존 runtime에서 bounded 보충하고, 남으면 이유를 명시한다. 승인5000장을 자동 보장하지 않는다.
2. 원본 입력·runtime·raw·native annotation·성공 ledger·실패/보충 근거를 보존한다.
   v132 성공 raw는 v133 import 참조 대상이므로 기존 output을 삭제하거나 이름을 바꾸지 않는다.
3. 새 release/output에서 Ego 연속 위치 샘플링과 환경별 새 배치 생성 경로를 구현·검증한다.
4. sunny11/sunny15/foggy11/foggy13/foggy15에 각각 새5K를 생성하고 순차 수집한다.
   환경 순서는 운영 편의에 따라 선택하되 실제 MR UI 날씨·시간을 매번 확인한다.
5. 전체30K의 실제 coverage·위치 분포·가시성 및 모델/mission rosbag 근거로 Sunny13 유지 여부를 판단한다.
   구버전이라는 이유만으로 재수집하지 않는다. 필요하면 일부 추가 수집을 먼저 검토한다.
   교체하더라도 기존 성공분을 먼저 삭제하지 않는다.

이 합의는 과거의 '같은5K geometry를6환경에 복사' 계획보다 우선한다.
현재 `environment_suite`의 Sunny13 입력/대시보드 연결은 유지한다. 다른5환경의 기존 복사 manifest는
과거 재현용이며, 새 생성·검증을 마치기 전 다음 본수집 입력으로 사용하지 않는다.

## 현재 입력과 공간 다양성 근거

- 입력 root: `~/morai_cap/base5_v2_261008/final_appearance_5k_v132_release`.
- 현재 입력: `environment_suite/sunny13/manifest.json`.
- SHA256: `89b053e7332e20f284c2de28d2f2365a3a9cf157d017c628481d9715ec5a1a77`.
- 현재 수집 output: `collection_5k_final_v132_sunny13_recovered_v133`.
  원래 `collection_5k_final_v132_sunny13`과 frozen/runtime 백업도 보존한다.
- 아래는 생성 코드와 `scenarios_5k/*_metadata.json`의 읽기 전용 조사다.
  실제 완료 raw 수, 학습 충분성 또는 모델 과적합 판정이 아니다.

| 조사 항목 | v132 입력에서 확인한 값 |
|---|---:|
| MGeo guided 주행 경로 길이 |2184.612m|
| 범용 route_general 기준점 bank |192개|
| 경로를192등분한 구간 폭 |11.378m|
| 인접 기준점 arc 간격 min / p50 / p95 / max |7.394 / 11.292 / 14.678 / 15.220m|
| 범용 장애물770셀의 사용 base XY / 최종 Ego XY |98 / 344개|
| 범용 사람·동물·복합830셀의 사용 base XY / 최종 Ego XY |98 / 377개|
| 위 두 범용군의 Ego 횡 offset 절댓값 p50 / max |0.15 / 0.30m|

경로는 `2026_molit_comp_global_path.txt`를 MGeo link에 연결한 생성 경로이며 지도 전체 도로 길이가 아니다.
기준점은 전체 경로에 층화되어 있다. 범용 비차량은 차량 전용 고주로·로터리와 별도 음영 구역을
제외하므로192개 모두를 쓰지 않는다. 사용 기준점의 최대 arc 공백 약899.916m를
그 구간 전체의 학습 누락으로 해석하지 않는다. 해당 미션/구역은 별도 카테고리가 담당한다.
98개 base의 재사용과 객체 거리·yaw·조합의 다양성은 동시에 존재한다.
최종 XY 개수 증가만으로 독립 배경 수나 충분성을 인증하지 않는다.

## 재사용할 구성과 환경별로 새로 뽑을 값

| 항목 | 다음5환경 정책 |
|---|---|
| 패밀리별 범용/미션 비율 |현재5K와 동일|
| 주요 자산 종류·할당 수 |현재 동결 spec의 primary 할당 보존; 희귀 자산 완전 랜덤 재추첨 금지|
| 거리 bin·방향·근접·객체 수 예산·미션 단계 |quota 및 의미 보존|
| 실제 Ego XY |허용 구간 안에서 환경별 새 층화 연속 샘플링|
| 실제 객체 거리·XY·허용 yaw·간격 |각quota 안에서 환경별 새 샘플링|
| 동반 객체·가림 관계 |현재 pool/예산/구역/가시성 제약 안에서 새 샘플링|
| NPC 경로·고주로 despawn 미션 제약·횡단축 |유지; 실제 경로에 임의 yaw/횡 jitter를 추가하지 않음|
| GT·지면 초기화·Park·CollisionData·resume |현재 계약 유지; 새runtime 검증 및 새output 사용|

유지할5K 장면 수는 고주로740범용/600미션, 로터리350미션, 기타차량80범용/50미션,
장애물770범용/600미션, 사람·동물·복합830범용/250미션, 독립음영500, 배경230이다.
122종 primary, 범용30/50/20 개수예산, 단일정지Ego/1capture, scale1,
실제 외관 이동 사거리154/음영24셀, 4class·7D·OD velocity OFF를 유지한다.
primary 할당 보존은 같은 위치·객체 조합·정확한 각도까지 복사한다는 뜻이 아니다.

## 구현할 경로

### 범용 Ego 위치

- `generate_base5_independent.py::anchor_bank`의 고정192개 최종 좌표 재사용을 새 opt-in 정책에서 대체한다.
  현재 code hash에 묶인 Sunny13 재현·resume 경로는 별도 frozen runtime으로 보존한다.
- 허용 도로의 유효 arc interval을 먼저 구성한다. 구역 제한·split·Ego footprint·요청 거리의
  배치 가능성을 함께 고려하며, 제외 구간을 포함한 전체경로 uniform rejection만으로 배분하지 않는다.
- 구간별 quota를 먼저 배정한 뒤 그 안의 위치를 연속 샘플링한다. 각 환경에 같은 구간 균형을
  유지하되 실제 arc 위치는 새로 뽑는다. 약11m는 기존 밀도의 참고값이지 새 고정192좌표 계약이 아니다.
- base XY, 최종 Ego XY/arc/link, 횡 offset/yaw, 공간 구간 ID와 재시도 이유를 기록한다.
  offset·회피 geometry 뒤 최종 XY 높이를 재계산하고 기존 도로 지지/실제 정지 검사를 유지한다.

### 미션과 이동 경로

고주로는 기존 연속 경로·차선·거리/추월 단계, 로터리는 접근/대기/진입/원형/탈출 구간을 유지한다.
정적 회피는 경계 간격·좌우·approach/entry/alongside/clearing을 유지하며 구간 안에서 새로 배치한다.
사거리 미션은 검증된 두 횡단보도와 endpoint 높이 lookup domain 안에서 관측 위치를 새로 선택한다.
음영은 외부 접근·내부·출구 quota, 전방 joint ROI 및 오른쪽/왼쪽 prior를 유지한다.
새 XY에 기존 live 접지·가시성 witness를 복사해 인증하지 않는다. 특히 음영 이동은 새 경로의
독립 지면 준비가 필요하며, 수동 높이 lookup을 domain 밖으로 외삽하지 않는다.

### 환경 생성과 식별

- `prepare_base5_environment_suite.py`는 현재 scene을 deepcopy하고 환경 metadata만 바꾼다.
  새 경로는 quota/primary 중심의 구성 명세를 재사용하고 scene payload를 환경마다 생성해야 한다.
- `frozen_specs_path`에 저장된 row seed를 그대로 두고 config seed만 바꾸면 새 배치가 되지 않는다.
  환경별 geometry seed와 retry seed를 분리해 명시하고 재생성 결과가 결정론적인지 검사한다.
- 논리적 quota slot ID, 실제 geometry scene ID, 환경 tag, source/version/hash를 구분한다.
  같은slot의 새 배치를 동일geometry weather A/B라고 부르지 않는다. token 충돌/동일payload 복사도 검사한다.
- 각 환경에서 검증을 마친 입력은 동결한다. resume/부족분 보충은 해당 환경의 입력·코드 계약만 사용한다.
  현재release/default config/suite/output을 덮어쓰지 않는다.

## Split과 기존 Sunny13 재사용

현재 Sunny13 생성 split4255/745를 유지한다. 원래 ledger와 metadata를 수정해 과거 split을 덮어쓰지 않는다.
다른 환경은 같은 도로/공간 group의 split 규칙을 공유하고, 주요 자산별 train coverage도 확인한다.
같은 quota slot의 split을 유지할 경우에는 후보 위치도 해당split 허용 구간 안에서 뽑는다.
반대로 slot번호만 같다는 이유로 다른도로의geometry를 동일장면으로 취급하지 않는다.

현재 split은 base link 기준이며 최종 Ego link/인접100m 시야의 중복 한계가 이미 있다.
새 샘플러에서 최종 link와 시야 중복을 감사하고 기존 한계를 숨기지 않는다.
30K split 장수는 실제 새manifest/변환 결과로 재계산한다. 기존25530/4470은 같은geometry 반복의
계획값이며 새배치30K의 확정 장수로 인용하지 않는다. 약85/15 목표와 rosbag dev-val/독립test는 유지한다.
`merge_base5_environment_infos.py`의 현재 group 검사만으로 공간 누수까지 검증했다고 주장하지 않는다.
새 식별·group 의미와 converter/merge의 호환 검사를 함께 준비한다.

## 종료 후 구현 검증 체크리스트

- [ ] Sunny13 종료/Park/collector 종료, 승인·부족·보충 및 원본 보존 확인.
- [ ] [점수제의 실제 학습 분포](261009_base5_training_split_visibility_and_temporal_decisions.md#수집-후-점수제에-따른-실제-학습-분포-점검)를
      자산·크기·거리별로 점검한다. 최종 crop 전에는 잠정값이며 유효 GT 기준으로 재확인한다.
      편중이 확인될 때만 다음 환경의 보조 자산 선택을 조정하며 현재 raw/점수제를 임의 폐기하지 않는다.
- [ ] 환경별 새geometry seed·연속 Ego 샘플러·식별/split 경로 구현; 현재입력 재현 경로 보존.
- [ ] CPU 단위검사: 구간 배분/허용 구역/동일seed 재현/환경간 새geometry/primary quota 보존/
      split 일관성·token 중복/최종XY높이/부족분 보충·완료resume 계약.
- [ ] 사전 선정 소량 live: 범용 장애물·사람·혼합·배경, 원거리/근접 좌우, 고주로·로터리·
      사거리·음영을 포함한다. 새관측 높이·실제정지·접촉·GT·가시성·raw완결 및 비용을 확인한다.
- [ ] 소량 pause/resume/완료resume와 bounded 보충을 새output에서 확인한다. 성공raw 추가촬영/변조 없음.
- [ ] 환경별5K offline 전수: primary/family/거리/방향/개수/phase quota, 위치의 구간별 분포,
      반올림XY 중복과 최근접거리/arc분포, 기존Sunny13 대비 반복 위치를 집계한다.
      최소위치간격을 억지로 강제해 중요미션밀집을 없애지 않는다.
- [ ] 각 환경의 manifest/runtime/reference/seed/식별/split/예상노출·ETA·용량 snapshot을 동결한다.
      기존v132 10738×6 노출표는 새30K 확정값으로 재사용하지 않는다.
- [ ] 새release 런북·dashboard suite를 연결하고 UI환경 확인 후 나머지환경 본수집을 시작한다.

기존 반복 비용 최적화는 [별도A/B](261009_base5_collection_cost_and_next_experiments.md)를 따른다.
geometry 변경과 runtime 속도 최적화를 하나의 인과 A/B로 섞지 않는다.
crop/BEVPool/초기화·Vast 학습 준비는 별도계획을 유지하며, 이 문서 갱신은 GPU 실험 승인이 아니다.

## 30K 후 Sunny13 판단

환경별 자산·거리·방향·선명한관측·도로구간 coverage와 mission rosbag 성능을 확인한다.
Sunny13 위치 반복이 실제 부족으로 이어지는지 판단하고, 충분하면 그대로 학습에 쓴다.
부족하면 해당 조건 추가 수집 또는 Sunny13 새5K 교체를 선택한다. 다른25K에서의 새배치도
같은지도/좁은미션구간을 공유하므로 완전히 독립적인 배경30K나 성능상승을 보장하지 않는다.

이 계획의 구현·실캡처 검증·나머지25K 생성은 아직 수행하지 않았다.
