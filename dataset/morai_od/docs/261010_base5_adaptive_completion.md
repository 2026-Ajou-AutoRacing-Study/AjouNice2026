# Base5 Sunny13 — 실패 원인별 자동 보충

2026-10-10. 입력 v132 / 실제 캡처 runtime v133을 유지한다. 이 문서가 v135의
수동 8셀 실행 안내보다 우선한다. 기존 성공·실패 raw와 과거 journal은 보존한다.

**2026-10-10 이후 최종 최신:** 사용자가 exact 실패 셀 복원 제약을 풀도록 승인했다.
[유연 대체 v140](261010_base5_flexible_deficit_replacement.md)에서 기존4,974장을 보존하고
새26장을 취득해 **5,000장/부족0**으로 완료했다. 아래v139 4,974/26은 strict 보충 종료 당시 이력이다.
최종목록/8075는v140이며 QA·원래source·부모journal/raw는 변경하지 않았다.

## 최종 결과 — 2026-10-10 / v139

**자동 보충과 실제 이동 계약 후검사까지 종료했다. 유효 선택은 4,974장, 부족은 26장(0.52%)이다.**
최종 canonical root는 release 아래 `collection_5k_sunny13_motion_contract_v139`다.
v137의 4,975장은 후검사 이전 수치이며 최종 학습 목록으로 사용하지 않는다.
대시보드 8075도 v139를 관측한다. collector는 종료했으므로 MR을 꺼도 된다.

| 단계 | 유효 선택 / 부족 | 근거 |
| --- | --- | --- |
| 사용자 종료 v135 | 4,948 / 52 | 기존 성공 행 검증 import |
| 자동 3회차 v137 | 4,975 / 25 | 115 outer 시도, 회차별 복구 17 / 7 / 3 |
| 실제 이동 계약 후검사 v139 | **4,974 / 26** | 이동이 정지로 대체된 1셀 제외, 실제 이동 재시도 3회도 미복구 |

- 추가 보충의 **순증은 26장**이다. v137 active 시간은 약48.5분이며 본수집 전체 시간이 아니다.
  정적 one-shot 111세션에서 Park 종료를 확인했다. 새 비calibration raw 116개 중
  동일 payload 중복 및 과거 payload 재촬영은 0이었다(v137 검증 범위).
- 남은 26셀의 대표 원인을 서로 배타적으로 묶으면 primary 가시성16, 정지 차량 위치4,
  보조 객체 가시성3, ground+가시성1, runtime 객체 누락1, 실제 이동 계약1이다.
  여러 사유가 함께 기록된 셀도 있으며 낮은 객체 센서 사각 등을 QA 완화로 통과시키지 않았다.
- `b5v2_02912`는 원래 standing police1 + walking Man6였으나 내부 quality 대체 후보가
  standing 경찰 2명으로 바뀌었다. spec/객체 수만 같은 것으로 이동 복구를 인증하면 안 된다.
  `run_base5_motion_contract_repair.py`는 **원래 primary의 실제 gait와 이동 mode별 최소 인원**을
  생성 후보와 최종 선택 양쪽에서 검사한다. 보조 객체가 추가로 움직이는 것은 정상 다양성이다.
  실제 이동 부족3셀 중 2셀은 복구했고 이 1셀은 끝내 남겼다.
- v138은 이동 인원을 정확히 같게 요구한 과도한 초기 pilot이다. 이동 보조 객체가 늘어난
  유효2셀은 v139에서 그대로 보존했다. v138 raw/journal/frozen driver는 이력으로 보존하며
  현재 driver로 강제 resume하지 않는다. v139도 code SHA가 고정되어 수정 후 강제 재개 금지다.
- 원래 v135의 **4,948행 전부 그대로 보존**했다. 원본 source/runtime, 부모 journal/raw도 보존한다.
  최종 ledger에는 이 행들이 이미 포함되어 있으므로 과거 ledger를 이어 붙이지 않는다.
- 단위 검사 **490개 통과**. 완료 후 `--resume --run` 검증에서 추가 시도0·raw0,
  선택 SHA·active 시간·부모 hash 불변을 확인했다. 상한은 반복 명령으로 초기화되지 않는다.
- 최종 CPU info: **train4,235 / val739 / test0**, 유효 GT10,921개
  (Vehicle4,504 / Pedestrian2,934 / Obstacle3,215 / Animal268).
  train primary122종, val117종, 최소 train primary2장, scene-group split 중복0,
  token 중복0, 입력 파일24,870개 누락0이다. accepted-visible primary floor 결손0이며
  coverage 전체 상태는 5K 미달이므로 `PARTIAL`이다.

결과 파일은 v139의 `validation_report.json`, `resume_validation.json`, `coverage_final.json`,
`remaining_diagnosis.json`, `quota.json`, `exclusions.json`, `selected_ledger.jsonl`이다.
`cpu_infos_final`은 **현재 320 crop/가시성 정책으로 검증한 임시 변환본**이며 최종 crop 확정 후
보존 raw에서 다시 변환한다. 이것은 모델 성능 인증이나 최종 학습 configuration 확정이 아니다.

### 다음 실행과 판단

부족26셀을 동일 명령으로 무한 반복하지 않는다. 가시성 증거가 없는 장면을 승인하거나
5,000장으로 표시하지 않는다. 이번 4,974장을 보존하고 다음 crop·투영·runtime 최적화 및
남은5환경의 연속 Ego 위치 다양화로 진행하는 것이 권장된다. 다른5환경의 새배치5K / 총30K
계획은 유지하되 실제 최종 승인 수와 부족분은 별도로 보고한다. 이후 이동 보충에도 실제 gait
감사를 적용한다. Sunny13 전체 재수집 필요성을 보여주는 무결성 오류는 현재 검사에서 발견되지 않았다.

최종 선택을 촬영 없이 재검증할 때:

```bash
ajou
python3 -u dataset/morai_od/scripts/run_base5_motion_contract_repair.py --resume
```

아래 v137 실행 설명과 초기 pilot 수치는 구현·진행 이력이다. 현재 완료 상태에서 재실행해도
추가 촬영은 없다. 새로운 부족분 재설계는 별도 검토·output 없이 강제 시작하지 않는다.

## 출발점과 변경 이유

- 본수집 4,930장, 과거 보충 3장, v135의 새 보충 15장으로 **4,948장 / 부족 52셀**이다.
- v135는 완료된 50시도를 보존한다. 부족 52셀 중 36셀은 해당 보충에서 미시도였고,
  16셀은 정적 2후보 상한을 소진했다. 남은 실제 이동 셀은 3개다.
- 매번 `--stop-after-cells 8`을 수동 실행해도 이미 소진한 셀은 복구되지 않는다.
  같은 payload 재촬영이나 무제한 재귀 대신, 명시적인 별도 추가 캠페인을 구현했다.

## 자동 진행 계약

현재 출력은 release root의 `collection_5k_sunny13_adaptive_v137`이다.
`run_base5_adaptive_completion.py`가 원본과 v135를 검증해 4,948행을 그대로 import한다.

| 원인 | 후보 선정 / 실제 촬영 |
| --- | --- |
| primary 센서 근거 부족 | 같은 거리·방향 계약 안에서 다른 상대 위치·yaw 후보를 선정 |
| 보조 객체 가림 | primary뿐 아니라 모든 parent의 가시성 proxy를 점수화 |
| runtime 객체 누락 | 객체 간 간섭이 적은 배치 우선, 실제 생성 여부 재검사 |
| 높이 의심·정지 차량 위치 변화 | 지도 지지·높이 변화가 작은 후보 우선, 기존 실제 검사 유지 |
| 실제 이동 객체 | 정적 one-shot 금지, 기존 지면 준비·이동 quality campaign 사용 |

- 셀당 **이 추가 캠페인에서 최대 3회차**다. 과거 시도는 삭제하지 않고 근거로 사용한다.
  `resume`으로 추가 3회차가 초기화되지 않는다. 승인된 셀은 이후 회차에서 제외한다.
- 한 번의 후보 선정은 최대 1,024 proposal에서 동일 계약의 유효 후보 32개를 비교한다.
  기존 실패 시야와 거의 같은 후보와 이미 사용한 scenario SHA를 제외한다.
- primary, 전체 spec(보조 객체 수 포함), split, scene group, 환경·거리·방향 계약을 유지한다.
  이동 quality loop 내부 후보가 전체 spec을 바꾸면 승인 raw가 있어도 원래 셀에 자동 합산하지 않는다.
- proxy는 배치 후보 순위일 뿐 실제 가시성 인증이 아니다. native GT와 기존 QA로만 승인한다.
  GT를 만들거나 가시성·높이·CollisionData·정지/Park 기준을 완화하지 않는다.
- 통신/안전 종료/증거가 불명확하면 전체 중단한다. 이미 완료된 child는 증거를 검증할 뿐
  다시 실행하지 않는다. 완료 증거가 없는 중단 child도 무조건 재로드하지 않는다.
- 기본 실행 예산은 20분이며 **현재 시도와 안전 종료를 마친 경계**에서 pause한다.
  특히 이동 준비가 있으면 hard deadline보다 늦어질 수 있다. 동일 명령으로 이어간다.
- 3회차 종료 후 부족분이 남으면 `complete_with_deficit`이다. 반복 실행으로 5K 달성을
  가장하거나 상한을 초기화하지 않는다. 계속 실패하는 조건은 별도 재설계 대상으로 남긴다.

## 명령어

현재 release에 맞는 기본 경로가 있으므로 긴 manifest/import 명령이 필요 없다.
ROS master, guarded bridge, MR Capture Mode와 Sunny13을 유지한다. 수집기가 이미 있으면
중복 실행하지 않는다.

```bash
ajou
python3 -u dataset/morai_od/scripts/run_base5_adaptive_completion.py --resume --run
```

추가 촬영 없는 검증은 같은 명령에서 `--run`을 뺀다. 사용자 `PAUSE`/`STOP` 파일은
자동으로 삭제하지 않는다. `complete`/`complete_with_deficit`에서 `--run --resume`을
다시 입력해도 새 회차나 촬영이 생기지 않는다.

최신 승인 목록은 새 출력의 `selected_ledger.jsonl` 하나다. 이미 원본 4,948행이 있으므로
원본·v135 ledger와 다시 연결하면 안 된다. `quota.json`과 `deficit_manifest.json`은 잔여 셀이다.
과거 v136 출력은 촬영 없는 import 준비 이력이며 현재 코드로 강제 resume하지 않는다.

## 대시보드

현재 8075는 최종 motion contract v139를 관측한다. 수집 코드나 journal을 고치는 기능은 없다.
필요할 때 **기존 대시보드만 종료한 뒤** 다음을 실행한다.

```bash
ajou
BASE5_ROOT="$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release"
python3 -u dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$BASE5_ROOT/collection_5k_sunny13_motion_contract_v139" \
  --manifest "$BASE5_ROOT/environment_suite/sunny13/manifest.json" \
  --suite "$BASE5_ROOT/environment_suite/suite.json" \
  --storage-root "$BASE5_ROOT/collection_5k_final_v132_sunny13" \
  --storage-root "$BASE5_ROOT/collection_5k_final_v132_sunny13_recovered_v133" \
  --storage-root "$BASE5_ROOT/collection_5k_sunny13_supplement_v134b" \
  --storage-root "$BASE5_ROOT/collection_5k_sunny13_supplement_v135" \
  --storage-root "$BASE5_ROOT/collection_5k_sunny13_adaptive_v137" \
  --storage-root "$BASE5_ROOT/collection_5k_sunny13_motion_contract_v138" \
  --port 8075
```

원본 raw 참조 경로를 storage root로 합산하며 hardlink는 중복 계산하지 않는다.
현재 환경은 manifest hash로 suite에 대응하고 승인 분포는 canonical 선택만 합산한다.

## 검증과 한계

- 첫 실제 정적 pilot 8셀에서 3셀 추가 승인, 4,951장 / 부족 49셀이다.
  복구 셀은 `b5v2_00507`, `b5v2_01241`, `b5v2_01548`이다. 이는 초기 pilot 결과다.
- 당시 read-only coverage 검사에서 **122종의 accepted-visible primary 최소 quota 결손 0**,
  unknown source 0, 오류 0이었다. 장면 수가 아직 5,000 미만이므로 완료 상태는 `PARTIAL`이다.
- 낮은 객체의 근접 사각지대나 먼 거리 센서 해상도 부족은 후보 순위만으로 해결되지 않을 수 있다.
  두 센서 근거가 모두 없는 primary를 빈 배경으로 바꾸거나 정답으로 강제 포함하지 않는다.
- 터널 일부 ground-only 실패에는 추정 plane의 지붕/원거리 오인 가능성이 있다. 지도 높이는
  비슷한데 plane이 몇 m 높게 예측된 사례를 발견했지만, 원인은 아직 확정하지 않았다.
  이 값을 물체가 실제로 몇 m 지면 아래에 있었다는 증거로 해석하지 않는다.
- 실제 이동 3셀은 이 초기 pilot에는 포함되지 않는다. 최종 결과는 문서 상단 v139를 따른다.
- 추가 자동 진행과 종료/resume/CPU 변환은 종료했으며 최종 결과는 문서 상단에 기록했다.

기존 production E25, tracker/planning/localization, raw/native annotation은 변경하지 않는다.
남은 5환경의 새 Ego 공간 배치와 30K 계획도 이번 보충으로 변경하지 않는다.
