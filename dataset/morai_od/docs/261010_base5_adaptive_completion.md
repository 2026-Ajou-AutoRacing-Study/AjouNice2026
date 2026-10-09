# Base5 Sunny13 — 실패 원인별 자동 보충

2026-10-10. 입력 v132 / 실제 캡처 runtime v133을 유지한다. 이 문서가 v135의
수동 8셀 실행 안내보다 우선한다. 기존 성공·실패 raw와 과거 journal은 보존한다.

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

현재 8075는 adaptive v137을 관측한다. 수집 코드나 journal을 고치는 기능은 없다.
필요할 때 **기존 대시보드만 종료한 뒤** 다음을 실행한다.

```bash
ajou
BASE5_ROOT="$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release"
python3 -u dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$BASE5_ROOT/collection_5k_sunny13_adaptive_v137" \
  --manifest "$BASE5_ROOT/environment_suite/sunny13/manifest.json" \
  --suite "$BASE5_ROOT/environment_suite/suite.json" \
  --storage-root "$BASE5_ROOT/collection_5k_final_v132_sunny13" \
  --storage-root "$BASE5_ROOT/collection_5k_final_v132_sunny13_recovered_v133" \
  --storage-root "$BASE5_ROOT/collection_5k_sunny13_supplement_v134b" \
  --storage-root "$BASE5_ROOT/collection_5k_sunny13_supplement_v135" \
  --port 8075
```

원본 raw 참조 경로를 storage root로 합산하며 hardlink는 중복 계산하지 않는다.
현재 환경은 manifest hash로 suite에 대응하고 승인 분포는 canonical 선택만 합산한다.

## 검증과 한계

- 첫 실제 정적 pilot 8셀에서 3셀 추가 승인, 4,951장 / 부족 49셀이다.
  복구 셀은 `b5v2_00507`, `b5v2_01241`, `b5v2_01548`이다. 이는 초기 pilot 결과다.
- 당시 read-only coverage 검사에서 **122종의 accepted-visible primary 최소 quota 결손 0**,
  unknown source 0, 오류 0이었다. 장면 수가 아직 5,000이므로 완료 상태는 `PARTIAL`이다.
- 낮은 객체의 근접 사각지대나 먼 거리 센서 해상도 부족은 후보 순위만으로 해결되지 않을 수 있다.
  두 센서 근거가 모두 없는 primary를 빈 배경으로 바꾸거나 정답으로 강제 포함하지 않는다.
- 터널 일부 ground-only 실패에는 추정 plane의 지붕/원거리 오인 가능성이 있다. 지도 높이는
  비슷한데 plane이 몇 m 높게 예측된 사례를 발견했지만, 원인은 아직 확정하지 않았다.
  이 값을 물체가 실제로 몇 m 지면 아래에 있었다는 증거로 해석하지 않는다.
- 실제 이동 3셀은 아직 이 초기 pilot 결과에 포함되지 않는다. 별도 live 결과로 갱신한다.
- 추가 자동 진행과 종료/resume/CPU 변환 결과는 완료 후 아래에 기록한다.

기존 production E25, tracker/planning/localization, raw/native annotation은 변경하지 않는다.
남은 5환경의 새 Ego 공간 배치와 30K 계획도 이번 보충으로 변경하지 않는다.
