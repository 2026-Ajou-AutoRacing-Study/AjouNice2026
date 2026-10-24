# Base5 데이터와 수집의 현재 기준

문서 상태: CURRENT. 데이터 근거 확인: 2026-10-21. 문서 정리: 2026-10-24.
현재 지원 입력은 **30,750장 / train 26,136 / val 4,614**다.
정규 Sunny/Foggy 11·13·15시 30,000장과 Sunny 장애물 추가 보강 750장을
**10개 원래 root/infos**로 연결한다. aggregate payload를 복제하지 않는다.
구 E24 Sunny13 5K, 부모 성공분, native 선택 5K를 다시 더하지 않는다.
모델·checkpoint·실제 학습 상태는 [모델 CURRENT](../../../docs/bevfusion/CURRENT.md)를 따른다.
데이터 준비·CPU 검사 통과는 학습·새 수집 시작 허가가 아니다.

<a id="data"></a>

## 승인 데이터와 실제 입력

| 지원 cohort | 장면 / train / val | 완료 근거 |
|---|---:|---|
| Sunny11 | 5,000 / 4,250 / 750 | [본수집·보충·전달](261018_base5_unified_generation_capture.md#sunny11-local-retrieved-261019) |
| Sunny13 v158a | 5,000 / 4,250 / 750 | [마지막 동물 장면·최종 ledger](261018_base5_unified_generation_capture.md#borrowed-terminal-animal-completion) |
| Sunny15 v159 | 5,000 / 4,250 / 750 | [최종 선택·JPEG·Vast 전달](261018_base5_unified_generation_capture.md#sunny15-training-package) |
| Foggy11 | 5,000 / 4,250 / 750 | [날씨 52장 교체·마감·전달](261019_base5_foggy13_reinforcement_and_capture.md#foggy11-weather-repair-261021) |
| Foggy13 기존 지원분 | 4,999 / 4,249 / 750 | [GT 0인 Bollard04 제외·원본 보존](261019_base5_foggy13_reinforcement_and_capture.md#foggy13-supported-training-export-261020) |
| Foggy13 별도 신규 WoodBox | 1 / 1 / 0 | [새 촬영·중복 0·Vast 복원](261019_base5_foggy13_reinforcement_and_capture.md#foggy13-terminal-one-30750-261021) |
| Foggy15 | 5,000 / 4,250 / 750 | [마지막 두 장·JPEG 회수](261019_base5_foggy13_reinforcement_and_capture.md#foggy15-terminal-two-and-jpeg-retrieval-261021) |
| Sunny11·13·15 장애물 각 250 | 750 / 636 / 114 | [3개 cohort의 native·split·전달](261019_base5_foggy13_reinforcement_and_capture.md#sunny-obstacle-supplement-750-261021) |

전체 전달·준비 receipt는 `artifacts/evaluation/base5_full30750_training_261021_v1/`의
`TARGET_READY.json`과 `FINAL_PREPARATION.json`이다. 10/21 준비 시점에 12 epoch·compile OFF·
일반 코사인 설정과 실제 19개 child/95개 경로의 Concat CPU 구성을 검사했다.
그 시점에는 학습 준비만 승인됐고 실제 시작은 승인하지 않았다. 이후 상태는 모델 CURRENT를 확인한다.

다음은 로컬 학습 입력 경로다. 모두 `artifacts/evaluation/` 아래이며 native raw 위치와 다르다.

| 입력 | evaluation 아래 경로 |
|---|---|
| Sunny11 | `base5_sunny11_training_retrieved_261019_v1/dataset/` |
| Sunny13 | `base5_sunny13_training_retrieved_261018_v1/dataset/` |
| Sunny15 | `base5_sunny15_training_export_261018_v2/sunny15/training3_jpeg_portable/` |
| Foggy11 | `base5_foggy11_training_export_261021_v3/qualified/foggy11/training3_jpeg_portable/` |
| Foggy13 기존 4,999 | `base5_foggy13_training_export_261020_v1/qualified_supported_v2/foggy13/training3_jpeg_portable/` |
| Foggy13 신규 1 | `base5_foggy13_terminal_one_261021_v1/training_export_v1/qualified/foggy13_completion/training3_jpeg_portable/` |
| Foggy15 | `base5_foggy15_training_retrieved_261021_v1/dataset/` |
| Sunny 보강 3개 | `base5_sunny_supplement_training_retrieved_261021_v1/<환경>/dataset/` |

최종 선택은 각 완료 출력의 `selected_ledger.jsonl` 하나이며, 학습은 실제 선택 기반
`layout_split_final/manifest.json`과 `derived_selected_ledger.jsonl`을 따른다.
Sunny15는 `base5_sunny15_completion_261018/last_cell_v1/`, 빌린 PC Sunny13는
`~/ajounice_base5_capture/collections/sunny13_5k_terminal_animal_v1/`가 최종 선택이다.
다른 환경의 전체 ledger·원격 원본·압축본·검증 SHA는 위 완료 근거에서 확인한다.

<a id="capture"></a>

## 생성·수집·복구 계약

운영 프로필 정리 후 과거 수집 JSON은 `configs/*/legacy/`에 보관한다. 이미 배포한 동결
bundle로 재개하며 기존 소스·SHA를 고쳐 쓰지 않는다. 옛 root 경로를 요구하는 수집기를
소스에서 재현할 때는 정리 전 `78d4f929` tree를 별도로 사용한다.

새 생성의 단일 진입점은 `scripts/portable_base5/generation.py`다. 6환경은 같은 공통 정책·
11단계·구현 SHA를 사용하며 환경과 seed만 새 identity로 기록한다.
현재 보강 프로파일은 `configs/dataset/base5_midrange_reinforcement_v163.json`이다.
파일 존재·CPU 생성·소량 native 검증만으로 본수집을 승인하지 않는다.
실행·수집·유연 보충은 [공통 실행 근거](261018_base5_unified_generation_capture.md),
Foggy13·후속 환경·750장 보강은 [마감 근거](261019_base5_foggy13_reinforcement_and_capture.md)를 따른다.

- 기존 v158a/v159의 배치·capture split 차이는 역사로 보존하며 재생성하지 않는다.
- 동결 scripts의 파일 목록·source/runtime·config·manifest·release SHA를 보존한다.
  새 도구는 공통 portable 경로에 추가한다. 구 output을 변경된 코드로 강제 resume하지 않는다.
- 고정 입력 전체 SHA는 시작·재개·종료 경계에서 확인하고 새 후보·native QA는 실제 검사한다.
  종료 integrity 실패·SIGKILL·Park 미확인을 정상 완료로 인증하지 않는다.
- 성공·실패 raw·부모 journal·소진한 시도·adopted mapping을 보존한다.
  부족분을 미검사 상태로 되돌리거나 부모 ledger를 중복 연결하지 않는다.
- ordinary 부족분과 통신·Park·CollisionData 무수신의 안전 중단을 구분한다.
  심각한 부유·실제 이동·충돌·Ego 정지 검사를 우회하지 않는다.
- 정적 full-RPY 본수집과 격리형 strict-direction 복구의 QA 경계를 유지한다.
  실제 이동은 최종 방향·실측 지지 영역·수동 높이의 두 waypoint 선형 이동을 따른다.
- SaveSensorData 수집과 UDP/ROS streaming을 혼동하지 않는다. 중복 collector·master·bridge를 실행하지 않는다.
  MR UI의 날씨·시간·annotation과 원형 전용 프로세스를 확인한다.
- 수집·MORAI 조작·전송·삭제·학습은 해당 사용자 승인 범위에서만 실행한다.

<a id="gt"></a>

## GT·원본 보존

native GT·parent/component·원래 vehicle/pedestrian/obstacle/animal 의미를 보존한다.
학습은 native full-RPY의 upright 7D enclosure와 Animal→Pedestrian의 3클래스 파생 정책이다.
OBJ 차량은 Vehicle이며 복합 component의 BBox를 부모 하나로 합치지 않는다.
실제 8Points를 치수표로 덮어쓰거나 scale을 재곱하지 않는다. 자산 eligibility는 [MR 자산 계약](MR_ASSET_COLLECTION_CURRENT.md)을 따른다.

RGB 학습 입력은 원본이 온전할 때 전수 검증한 **Q90 JPEG**다. mask/depth는 변환하지 않는다.
Q90은 손실 변환이며 PNG의 무손실 백업이 아니다. native GT·비RGB·실패·비선택 원본은 보존한다.
과거 승인된 PNG/cache 정리는 해당 완료 근거의 receipt만 따른다. 새 삭제 승인으로 확대하지 않는다.
빌린 PC PNG·native GT 보존 지시와 환경별 실제 삭제 0/삭제 완료 이력을 구분한다.
삭제된 RGB가 포함된 과거 SHA 증거는 고쳐 쓰지 않으며 완료 세션을 강제로 resume하지 않는다.
원문 Object velocity·trace는 진단 metadata이지 완전한 동기 velocity GT가 아니다.
현재 single-frame 7D OD에 velocity supervision을 넣지 않는다.
전방 80~100m LiDAR 0점 편차의 센서 변경은 [후속 검토](261019_base5_foggy13_reinforcement_and_capture.md#front-far-sensor-followup-261019)이며 현 데이터에 섞지 않는다.

## 이력과 재현

구 E24·Base1~4·초기 Base5·확대 중단·중간 부족분·과거 ETA는 [legacy 계보](legacy/README.md)에 격리했다.
정리 전 CURRENT의 세부 수치·경로는 [원문 snapshot](legacy/context_261024/BASE5_V2_CURRENT.before.md)에 보존했다.
명시적인 재현에만 당시 source·catalog·단위·hash를 선택한다. 과거 CURRENT/계획을 현재 값으로 복구하지 않는다.
