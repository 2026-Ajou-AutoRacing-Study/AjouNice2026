# Base5 생성·수집 공통 실행 정책

문서 상태: EVIDENCE. 기준: 2026-10-18 KST.
공통 구현·기존 데이터 인계·두 PC의 소량 native 검증을 완료했다. 준비 후 사용자의 추가 명시 승인으로
10/18 06:18 KST 두 본수집을 재개했다. 아래의 인계·자격시험 수치는 당시 고정 기록이며,
최신 실행 상태는 [본수집 재개 기록](#본수집-재개-기록)을 따른다.
이 문서는 새 5K 수집 완료나 전체 수집 시간의 인증이 아니다.

## 결정과 보존 범위

Sunny13와 Sunny15를 안전하게 일시정지한 뒤, 환경별 실행기 분기를 공통 실행기로 통합한다.
과거 source/runtime·원본·native GT·승인·부족분·복구 시도·Park 실패 이력은 수정하지 않는다.
새 출력의 schema-1 import가 기존 선택을 한 번만 참조한다. 부족분을 미검사 장면으로 되돌리거나
소진한 복구 횟수를 초기화하지 않는다. 준비 단계에서는 본수집을 자동 시작하지 않았으며,
이후 사용자가 두 PC의 실제 실행을 agent에 명시적으로 맡겨 재개했다.

인계 기준은 다음과 같다. 수집기가 실제로 `paused`를 기록하고 모든 현재 native 세션의 정지·P를 확인했다.

| 환경 | 보존할 직전 출력 | 승인 | 부족분 | 검사 완료 |
|---|---|---:|---:|---:|
| 빌린 PC Sunny13 | `collections/sunny13_5k_recovery_v2` | 1,313 | 15 | 1,328 |
| 로컬 Sunny15 | `~/morai_cap/base5_sunny15_v159_261017/collection_5k_after_crash_v1` | 557 | 3 | 560 |

Sunny15 원래 출력의 MORAI 종료·Park 미확인 실패는 그대로 남는다. 위 Sunny15 인계는 그 이후
정상 재개한 별도 출력이 안전 PAUSE된 상태다. 옛 실패를 정상 PAUSE로 고친 것이 아니다.

## 왜 시간이 늘었는가

Sunny15의 `direction_reference.configuration()`은 장면 하나를 복구할 때도 고정된 원본 5K의
`input_pins()` 전체와 연결 참조를 재귀 순회했다. parent도 일반 chunk/복구 전에 고정 입력 전체를
반복 확인했고, portable child는 runtime·release·원본 계약 검사를 반복했다.
기존 process-local SHA 캐시는 중복 파일 읽기는 줄였지만 JSON·경로·참조 순회까지 없애지는 못했다.
시간대 자체가 이런 검사를 필요로 하는 것은 아니다.

새 공통 실행은 다음을 구분한다.

| 대상 | 새 검사 시점 |
|---|---|
| 동결 시나리오·메타데이터·코드·설정·지도 등 고정 입력 | 시작·재개·종료에서 실제 전체 SHA 확인 |
| 고정 입력의 SHA·참조 목록·runtime import closure | 해당 부모 세션 안에서만 검증 결과 재사용 |
| 새 대체 시나리오·메타데이터·지면 준비 자료 | 생성·연결·검증 시 실제 검사 |
| 센서 원본·GT·캡처 audit·calibration·seed·session·Park·충돌 근거 | 기존 실제 검사 유지. 고정 입력 캐시 대상 아님 |

새 자료 검사에서는 이미 검증한 고정 참조 그래프를 다시 열지 않는다. 원형 full closure를
상위집합으로 유지하고 새 파일·새 참조만 실제 검사하는 delta 경로다. 과거 계약 대조에는
원형과 정확히 같은 mapping을 사용하며 delta를 적용하지 않는다.

부모는 sealed anonymous memfd와 실제 UID/PID/start ticks·실행 파일/모드를 통해 child에
세션 snapshot을 전달한다. 디스크의 임의 JSON에 `verified=true`라고 쓰는 방식은 허용하지 않는다.
타깃별 인자·장면 수·출력·복구 셀·seed 범위를 제한한다. direction child는 canonical module의
`main()`을 호출하여 `runpy`의 별도 `__main__` 함수가 최적화를 우회하지 못하게 한다.

고정 파일이 수집 중 바뀌면 그 즉시가 아니라 다음 경계에서 검출될 수 있다는 의도적인 절충이 있다.
수집 중 동결 입력을 편집하지 않는다. 종료 검사 실패는 `halted`와 별도 integrity-failed 근거로 남기며,
그 세션의 선택을 정상 인증하지 않는다. SIGKILL·프로세스 장애로 종료 검사를 못 했다면 정상 완료로
간주하지 않으며 재개 검사와 native 안전 근거가 필요하다. 바뀐 파일을 원복하면 항상 탐지한다는
주장도 하지 않는다. 원격에서는 동결 bundle·기존 output·실행 코드 overlay를 namespace의 읽기 전용으로 둔다.

## 두 환경의 공통 수집 정책

새로 찍는 부분은 같은 `unified_capture.py`와 원형 static-attitude parent를 사용한다.
정적 full-RPY 도로 지지 관계 판정과 이동 복합체 최종 방향·넓은 지면 지지 준비를 모두 유지한다.
가시성·native 크기·실제 이동·심각한 부유·충돌·통신·정지·P 기준을 완화하지 않는다.
기존 Sunny13 성공분을 새 QA로 승인했다고 소급 해석하지 않는다.

원격은 원래 427개 runtime 파일을 고치지 않고, 공통 파일 11개를 별도로 추가한다.
전체 438개는 dataset scripts 371개와 모델 관련 67개다. 새 읽기 전용 overlay에는 전자만 복사하고,
모델 관련 67개는 기존 bundle의 읽기 전용 파일을 그대로 사용한다.
새 overlay와 portable helper는 `work/unified-runtime-v1`에만 배포한다. 기존 ROS·guarded bridge·
bundle·시스템 설치·사용자가 보호한 TXT는 수정하지 않는다. 원래 release는 source의 기존 자격이고,
새 실행 정책의 실제 소량 검증은 별도 자격이다. 옛 release가 새 실행기까지 승인했다고 쓰지 않는다.

## 생성 통합과 기존 배치의 차이

새 생성의 단일 진입점은 `scripts/portable_base5/generation.py`다. Sunny/Foggy 11/13/15의
여섯 환경이 같은 v160 정책·11단계를 사용한다. 환경과 seed는 새로운 배치의 identity일 뿐
정책 선택기가 아니다. 차선 배치 가능성 선별·원거리 보조 차량 fallback·capture split 고정·
전구간 후보 및 known-map layout overlay를 공통 적용한다. 기존 중간 단계 생략 경로는 노출하지 않는다.
새 Sunny13 생성은 별도 출력과 `--allow-new-sunny13-plan`을 명시해야 한다.

기존 Sunny13 v158a와 Sunny15 v159의 동결 배치는 그대로다. 공통 파일은 같은 bytes지만,
Sunny15에는 배치 가능 차선 사전 선별·원거리 fallback과 전구간 후보 split 처리가 추가됐었다.
특히 Sunny13 정적 미션은 capture split에 따른 구간 제한이 남고 Sunny15는 전체 허용 구간을
양쪽 후보로 사용한다. 따라서 기존 두 배치가 공간적으로 완전히 같은 분포라고 말하지 않는다.
이 역사적 차이를 없애려고 이미 취득한 장면을 버리거나 나머지 고정 장면을 몰래 다시 생성하지 않는다.
후속 환경부터 단일 생성 정책·구현 SHA를 receipt에 기록해 분기가 재발하지 않게 한다.

새 생성 진입점은 동결 `dataset/.../scripts/*.py` 전수 inventory를 바꾸지 않는 별도 portable 위치에 둔다.
기존 release 검증기를 우회하거나 동결 runtime 해시를 고쳐 쓰지 않는다.

## 검증과 실측

구현 중 실측: 고정 full-manifest 목록 최초 계산 약 14.306초, 같은 세션의 재사용 약 0.761초.
21,849개 참조 중 native 등 재사용 대상이 아닌 자료는 여전히 실제 검사했다. 이 숫자는 반복 입력
검사 단계의 비용이며, 5K 총 시간이나 실제 촬영 전체가 같은 비율로 줄었다는 뜻은 아니다.

실제 직전 복구 장면 `...00589/.../replacement_deck_0/manifest.json`도 촬영 없이 대조했다.
원형 새 자료 목록 계산 5.685초 → delta 0.777초(약 86.3% 감소)였다.
원형 1,850개 pin이 새 21,854개 pin에 같은 SHA로 모두 포함됐다. 새 자료 7회 검사와 기존 참조
82회 재사용을 관측했다. 추가 pin은 이미 확인한 full-base 맥락이며 native QA를 삭제한 것이 아니다.

최종 회귀 검사는 portable 실행기와 기존 static-attitude·batch·direction·안전 관련 테스트를 함께
실행해 **1,074개 통과**했다. 입력 변경 검출·잘못된 연결·중복 승인 거부·인계 이력 보존·child 권한·
종료 검산 실패·완료 재생의 추가 촬영 방지를 포함한다. 두 쉘 wrapper의 `bash -n`도 통과했다.

두 PC에서 같은 최종 실행기 SHA
`3d64243e648e02686f4d7296b66e9a2b65583fe08801d00ffec4610ed414f522`로 검증했다.
정적 혼합 장애물·차량·정지 사람·교차로 이동 사람·음영 이동 복합체의 다섯 장면이다.
이동 복합체는 실제 strict-direction 지면 준비와 새 참조 연결 경로를 거쳤다.

| 실제 별도 자격시험 | 로컬 Sunny15 | 빌린 PC Sunny13 |
|---|---:|---:|
| native 승인 / 시험 촬영 | 5 / 5 | 5 / 5 |
| 첫 묶음 후 실제 PAUSE·재개 | 통과 | 통과 |
| 완료 후 재확인 | 추가 촬영·worker 실행 없음 | 추가 촬영·worker 실행 없음 |
| 자격시험의 활성 누적 시간 | 150.58초 | 202.95초 |
| 새 본수집 출력의 native 촬영 | 0장 | 0장 |

다섯 장면은 QA 경로를 확인하기 위한 불균등 표본이다. 5K의 시간 예측에 그대로 확대하지 않는다.
로컬 시험의 시작·종료 경계 전체 검산은 합계 50,756회 읽기·약 5.03GB·6.89초였다.
전체 원본 자격시험을 매번 다시 해석하는 대신, 그 입력·runtime·native 증거·gate 결과 파일의
실제 SHA를 경계에서 모두 확인한 경우에만 기존 source 자격 결과를 재사용한다.
새 실행기의 native 자격과 실제 촬영 QA는 이 결과로 대체하지 않는다.

구현 중 완료 재확인에서 이동 장면용 복구 프로세스가 다시 실행되는 경로를 발견했다.
추가 촬영 전에 시험을 차단하고 원형의 읽기 전용 native 검증 경로로 수정했다.
당시 실패한 시험 출력은 `halted` 그대로 보존하고, 별도 새 시험에서 위 결과를 얻었다.
기존 본수집 성공분을 잃거나 실패 status를 정상으로 바꾼 것이 아니다.

인계 시 로컬 native 세션 21개·완료 복구 7건, 원격 native 세션 187개·완료 복구 64건을 확인했다.
부모의 중첩 import와 이미 소진한 복구 이력도 보존했다. 원격 배포는 원본 runtime 427개를
덮어쓰지 않고 승인된 작업 폴더의 새 overlay를 사용한다.

기존 대시보드 ETA에는 과거 느린 구간의 누적 시간이 남는다. 재개 후 실제 승인 속도를 관측하기
전에는 16시간을 특정 새 시간으로 바꾸어 약속하지 않는다. native 준비·캡처·실제 재배치 비용은 남는다.

## 현재 출력과 실행 명령

| 환경 | 새 본수집 출력 | 공통 profile |
|---|---|---|
| 로컬 Sunny15 | `~/morai_cap/base5_sunny15_v159_261017/collection_5k_unified_v1` | `artifacts/evaluation/base5_unified_runtime_261018/sunny15_profile.json` |
| 빌린 PC Sunny13 | `~/ajounice_base5_capture/collections/sunny13_5k_unified_v1` | `~/ajounice_base5_capture/work/unified-runtime-v1/sunny13_profile.json` |

각 새 출력의 `unified_paused_handoff.json`, `imported_prefix.json`,
`unified_live_qualification.json`이 실제 인계·자격 근거다. 새 출력의 상태는
인계 직후에는 `prepared_waiting_for_resume`였으며 승인/부족분은 위 인계 표와 같았다.
현재 실행은 아래 재개 기록을 따른다. 별도 시험 장면은 본수집에 합산하지 않는다.
최초 profile은 main native 0장인 준비 단계에서만 수정했고 수정 전 profile도 보존했다.
이후 runtime 변경은 이 자격을 그대로 재사용할 수 있는 변경으로 간주하지 않는다.

MORAI·ROS·기존 guarded bridge는 그대로 유지한다. Sunny 시간·Capture Mode·카메라 1~4와
LiDAR 5 annotation ON을 확인한다. 이전 출력의 구 수집 명령을 동시에 실행하지 않는다.

로컬 Sunny15 — 사용자가 직접 실행:

```bash
cd ~/AjouNice2026
bash scripts/portable_base5/start_unified_local.sh --run --confirmed-environment sunny15
```

빌린 PC Sunny13 — 그 PC 터미널에서 사용자가 직접 실행:

```bash
bash ~/ajounice_base5_capture/start_base5_unified.sh run --confirmed-environment sunny13
```

두 wrapper 모두 실행 인자 없이 호출하면 촬영 없는 계획만 표시한다.
로컬 wrapper는 기존 로컬 master 주소 `http://127.0.0.1:11311`을 명시한다.
정상 일시정지는 별도 터미널에서 해당 새 출력에 `PAUSE`를 만든다.

```bash
# 로컬 Sunny15
touch ~/morai_cap/base5_sunny15_v159_261017/collection_5k_unified_v1/PAUSE
# 빌린 PC Sunny13
touch ~/ajounice_base5_capture/collections/sunny13_5k_unified_v1/PAUSE
```

현재 묶음 종료·실제 정지·P와 `paused`를 확인한 뒤 위의 같은 실행 명령으로 재개한다.
`halted`를 강제 우회하지 않는다. 수집기에 Ctrl+C를 보내면 native 안전 종료나 수동 확인이 필요한
중단이 될 수 있으므로 정상 일시정지 요청에는 `PAUSE`를 쓴다.
통신·Park 확인 전 MORAI·ROS·bridge를 끄지 않는다.

공통 실행기의 준비·자격시험 단계는 본수집을 자동 시작하지 않으며, 자동 `--complete`도 켜지 않는다.
5K 후 부족분 보충·Q90 변환·압축·전달은 별도 완료 단계다. 새 출력에 경로가 고정된 구 완료
스크립트를 무작정 실행하지 않는다. 원본 PNG의 자동 삭제 정책도 추가하지 않았다.

## 대시보드 전환

원격 MORAI crash 이후의 현재 대시보드는 아래 [안전 인계 절](#원격-morai-crash-이후-안전-인계)의
`recovered_dashboard.py`를 사용한다. 여기의 원격 `unified_dashboard.py --borrowed`는 이전 출력용 이력이다.

기존 웹 서버와 구 `watch`는 이전 출력의 PAUSE 상태를 계속 보여 준다.
해당 **웹 서버 터미널에서만** Ctrl+C를 누른 뒤 새 관측기를 수동 실행한다.
수집기·ROS·bridge를 종료하는 것이 아니다. 새 관측기는 사용 중인 포트를 강제로 비우지 않는다.

```bash
# 로컬 Sunny15: 저장소에서 실행
python3 -B scripts/portable_base5/unified_dashboard.py \
  --profile artifacts/evaluation/base5_unified_runtime_261018/sunny15_profile.json \
  --port 8075 --run

# 빌린 PC Sunny13: 그 PC에서 실행
python3 -B ~/ajounice_base5_capture/work/unified-runtime-v1/unified_dashboard.py \
  --borrowed --port 8075 --run
```

로컬 8075는 Sunny15, 빌린 PC 자체의 8075는 Sunny13이다. 기존 로컬 8076→원격 8075 SSH 터널은
유지할 수 있다. 관측기는 새 출력만 관측하며 원본·수집 상태를 수정하거나 캡처를 제어하지 않는다.
두 PC 모두 촬영·서버 시작 없는 계획 검사를 통과했다.

복사할 명령은 저장소의 `scripts/portable_base5/UNIFIED_COLLECTION_COMMANDS.txt`와 빌린 PC의
`~/ajounice_base5_capture/work/unified-runtime-v1/UNIFIED_COLLECTION_COMMANDS.txt`에 있다.
사용자가 편집을 금지한 기존 TXT의 보호 구역은 건드리지 않았다.

## 본수집 재개 기록

2026-10-18 06:18 KST, 사용자가 잠든 동안 두 PC의 본수집 실행을 명시적으로 요청해 위의
동일 명령을 실행했다. 코드·profile·QA·동결 source는 변경하지 않았다.
중복 수집기가 없고 기존 ROS·guarded bridge·Ego/CollisionData 수신이 정상임을 먼저 확인했다.
시작 당시 디스크 여유는 로컬 약 143GiB, 원격 약 220GiB였다.

06:20 KST 초기 진행 확인:

| 환경 | 재개 직전 승인 | 부모 진행 보고 승인 | 부족분 | 동작 |
|---|---:|---:|---:|---|
| 로컬 Sunny15 | 557 | 573 | 3 | 첫 신규 묶음 16/16 승인, 다음 묶음 촬영 |
| 빌린 PC Sunny13 | 1,313 | 1,329 | 15 | 첫 신규 묶음 16/16 승인, 다음 묶음 촬영 |

이는 확인 시점의 부모 진행 보고다. selected ledger의 주기적 동결 체크포인트와 같다는 뜻은 아니다.
64장 단위 저장 전에도 완료된 native 묶음의 실제 원본·ledger·Park 근거는 새 출력에 남는다.
진행 중 child를 부모 숫자에 다시 더하지 않는다. 5K 완료·부족분 보충 완료 인증은 아니다.

로컬은 별도 tmux 서버 `base5_unified`, 세션 `sunny15_261018_061755`에서 실행했다.
SSH·VS Code·이 대화의 연결 종료와 독립적으로 유지된다.

```bash
tmux -L base5_unified attach -t sunny15_261018_061755
```

로컬 stdout 로그는
`artifacts/evaluation/base5_unified_runtime_261018/sunny15_main_20261018_061755.log`다.
원격은 tmux가 없어 새 session으로 분리한 supervisor PID `2784150`에서 실행했다.
원격 stdout 로그는
`~/ajounice_base5_capture/work/unified-runtime-v1/operator_capture_sunny13_20261018_061823_1792271903664811751.log`다.
stdout은 진행마다 출력되는 로그가 아니므로, 최신 진행은 새 출력의 `status.json`과 native ledger를 본다.

기존 웹 서버와 구 watch는 사용자의 별도 수동 운영을 존중해 변경하지 않았다.
따라서 구 대시보드에 PAUSE가 보여도 새 본수집의 중단을 뜻하지 않는다.
새 출력의 웹 관측으로 바꾸려면 위 대시보드 전환 명령을 사용한다.
오류 시 안전 중단 정책은 유지하며, 상시 인력 감시·MORAI 자동 재시작을 추가한 것은 아니다.

## 원격 MORAI crash 이후 안전 인계

10/18 09:28 KST, 원격 Simulator가 Vulkan 렌더링 경로의 signal 11로 종료됐다.
`work/morai-session/config/unity3d/MORAI/Simulator/Player-prev.log`에 오류가 남았고,
Ego·Object·CollisionData가 끊겨 chunk 0170의 종료 P를 확인하지 못했다.
부모는 `Batch final Park unverified`로 halted했다. 고정 입력 경계 검산은 통과했고,
디스크 부족이나 입력 해시 불일치로 종료된 것이 아니다.

확정된 안전 prefix는 승인 2,688장·부족분 32장, 검사 완료 2,720장이다.
마지막 chunk의 8장 승인 후보와 실패 1장은 부모에 합산하지 않고 원본 그대로 격리한다.
이 8장의 품질이 나쁘다는 판정이 아니라, 묶음 종료 안전 근거가 없기 때문이다.
기존 halted·Park false·raw·선택·부족분·완료 복구 이력은 수정하지 않는다.

새 `unified_crash_import.py`는 정확한 checkpoint 다음의 stale/Park-false chunk 하나만
격리하고 안전 prefix를 schema-1로 새 출력에 인계한다. 다른 원인의 halted·임의의 unsafe
세션·진행 중 복구·구멍 난 prefix를 같은 명령으로 우회하지 않는다.
원형 native·static-attitude·완료 복구 verifier와 중첩 import를 실제로 대조하며,
새 native 촬영 없이 CPU 준비만 한다. 동일한 공통 수집기와 입력의 기존 5/5 자격은 모든
proof byte를 다시 검산한 뒤 원 보고서를 바이트 그대로 계승한다. 새 live 시험이라고 쓰지 않는다.

| 구분 | 경로 |
|---|---|
| 보존한 실패 출력 | `~/ajounice_base5_capture/collections/sunny13_5k_unified_v1` |
| 별도 복구 출력 | `~/ajounice_base5_capture/collections/sunny13_5k_unified_v1_after_crash_v1` |
| 별도 복구 profile | `~/ajounice_base5_capture/work/unified-runtime-v1/crash-continuation-v1/sunny13_profile.json` |
| CPU 인계 근거 | 새 출력의 `unified_crash_handoff.json`, `unified_crash_continuation.json` |

인계 준비 후 원격 터미널에서 사용자가 실행한다. Sunny13·Capture Mode·카메라 1~4와
LiDAR 5 annotation ON·Ego Network Connect를 확인하고 기존 ROS·guarded bridge를 유지한다.

```bash
bash ~/ajounice_base5_capture/resume_base5_collection.sh run --confirmed-environment sunny13
```

CPU `prepare`는 이미 준비한 출력에 다시 실행하지 않는다. 이 run은 원래의
`checks/unified_capture.py`를 새 profile로 실행하므로 수집·검증·복구 정책을 분기하지 않는다.
Ctrl+C의 정상 PAUSE 요청도 새 출력에만 연결한다. 복구 wrapper는 기존 실패 출력과 source
profile을 읽기 전용으로 보호하며, 예전 `start_base5_unified.sh run`으로 halted를 강제 재개하지 않는다.
현재 상태 확인은 다음 명령이다.

```bash
python3 -B ~/ajounice_base5_capture/work/unified-runtime-v1/recover_unified_borrowed.py status
```

실제 원격 CPU 인계는 완료했다. native 세션 193개·완료 복구 59건과 중첩 import를 검산했고,
원형 schema-1 verifier가 승인 2,688장·부족분 32장·기존 활성 누적 시간을 그대로 승인했다.
기존 출력은 `halted / Batch final Park unverified` 그대로이며 새 출력은
`prepared_waiting_for_resume`, 신규 native 세션 0개다. 기존 5/5 자격의 proof 629개를 다시 확인했다.
공통 수집기 SHA `3d64243e648e02686f4d7296b66e9a2b65583fe08801d00ffec4610ed414f522`는 변경하지 않았다.
신규 crash importer 50개·wrapper 49개를 포함한 전체 관련 회귀 1,173개와 쉘 문법 검사를 통과했다.
이번 작업에서는 명령 준비와 CPU 인계만 했으며 본수집을 자동 시작하지 않았다.

사용자가 운영 중인 8075 웹 서버는 종료하거나 교체하지 않았다. 기존 공통 대시보드는 이전
출력의 crash 이력을 보여 주므로 새 복구 출력용 읽기 전용 관측기를 별도로 제공한다.
기존 **웹 서버 터미널에서만** Ctrl+C 후 사용자가 다음 명령으로 교체한다.

```bash
python3 -B ~/ajounice_base5_capture/work/unified-runtime-v1/recovered_dashboard.py --port 8075 --run
```

`recovered_dashboard.py`는 새 profile·인계 receipt·원 자격 증거를 검증하고 동일한 동결 UI/HTML을
재사용한다. 새/기존 원본·MORAI·workspace는 읽기 전용이며 별도 observer history만 쓰기 가능하다.
수집 lock·PAUSE·캡처·기존 서버를 조작하지 않는다. 포트가 사용 중이면 종료를 강제하지 않고 거부한다.
중첩 인계의 원본 경로도 승인 분모와 함께 용량 추정에 포함하며 workspace/collections 밖의
임의 경로는 거부한다. 원격 8077의 실행 없는 계획 검사와 동결 backend의 `--once` 대조에서
정확한 새 출력의 `running / 승인 2,699 / 부족 32`를 확인했다. 기존 서버와 수집은 그대로 유지했다.
원격 TXT의 대시보드 명령도 갱신했다. 새 관측기·공통 관측기·원형 backend 관련 테스트 109개를 통과했다.

## Sunny15 정상 종료 후 부족분 보충

아래 `exact_v2` 설명과 명령은 당시 실행 기록이다. 현재는
[유연 보충](#flexible-completion)으로 전환했으므로 이전 명령으로 동시 취득하지 않는다.

로컬 `collection_5k_unified_v1`은 검사 5,000장·승인 4,941장·부족 59장·실행 오류 0으로
`complete_with_deficit` 종료했다. 단순히 기존 본수집 명령을 다시 실행하면 완료 기록을
재검증할 뿐 부족분을 새로 찍지 않는다. 사용자의 추가 수집 승인으로
`scripts/portable_base5/complete_unified.py`를 별도 진입점으로 준비했다.
기존 부모 status·ledger·원본·native GT·실패 기록은 변경하지 않는다.

별도 출력은 `artifacts/evaluation/base5_sunny15_completion_261018/exact_v2`다.
부족한 59개 quota만 새 geometry로 재생성하며 spec·primary·ordered actors·motion·yaw 규칙·
capture split·group·원형 크기·환경을 보존한다. 이미 실패한 payload는 내부 복구의 실제 촬영
직전까지 금지 목록을 전달한다. 기본 명령은 읽기 전용 계획이며 `--run`만 촬영을 허용한다.
각 quota의 추가 시도 상한은 기본 2회로 고정하고 재개 때 초기화하지 않는다.

새 보충은 **기존 isolated strict direction-reference recovery QA**를 사용한다. 일반 정적 chunk의
full-RPY 재분류 경로와 동일한 구현이라고 표현하지 않는다. 기존 strict 승인 기준을 완화하지
않으며 통신·CollisionData·정지/P 불명은 후속 장면을 중단한다. PAUSE·STOP·시간 예산은
현재 단일 셀의 native 안전 종료 후 다음 셀 경계에서 적용된다. 미완료 child나 `halted`를
일반 resume로 강제 재실행하지 않는다. 종료 무결성 실패는 선택 미인증으로 남긴다.

```bash
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311

# 읽기 전용 계획
python3 -B scripts/portable_base5/complete_unified.py \
  --profile artifacts/evaluation/base5_unified_runtime_261018/sunny15_profile.json \
  --output artifacts/evaluation/base5_sunny15_completion_261018/exact_v2

# 본수집 재시작이 아니라 부족분만 새로 취득한다.
# 첫 native 확인은 같은 명령에 --stop-after-cells 1을 추가한다.
python3 -B scripts/portable_base5/complete_unified.py \
  --profile artifacts/evaluation/base5_unified_runtime_261018/sunny15_profile.json \
  --output artifacts/evaluation/base5_sunny15_completion_261018/exact_v2 \
  --run --confirmed-environment sunny15
```

처음에는 기존 5K의 native QA를 전수 재구성하도록 준비했으나, 약 30분 동안 보충 촬영으로
진입하지 못했다. 사용자의 지적에 따라 불필요하게 범위를 확대한 이 검사를 중단했다.
현재는 **기존 승인 4,941장을 정상 종료된 확정 부모 ledger로 신뢰**한다. 승인/부족분 ID·개수,
source/runtime·guard·인계 receipt·기존 5셀 자격·정상 종료 근거와 SHA 불변성을 확인한다.
기존 4,941장의 픽셀·포인트·full-RPY·가시성을 다시 판정하거나 raw bitrot을 감사하지 않는다.
새 59개의 실제 strict native QA는 유지한다. 학습 전달본 생성 시 실제 raw·GT를 별도 검증한다.
최초 `exact_v1` 준비는 입력 그래프 미등록 오류로 native 0장·시도 0회에서 종료했으며 보존했다.
고정 입력 그래프만 한 번 등록하도록 고친 새 `exact_v2`를 사용하고 `halted`를 정상으로 고치지 않는다.

별도로 같은 약 3.985MB 분할 overlay의 반복 JSON 해석 비용도 확인했다.
동결 dataset 파일을 편집하지 않고 portable 범위에
`overlay_read_cache.py`를 추가했다. **호출할 때마다 실제 파일 SHA를 읽고**, 처음 보는
`(path, SHA)`만 원형 `read_overlay`로 전부 검증한 후 해석 결과를 재사용한다.
재귀 불변 dict/list, 일반 컨테이너로의 deepcopy, 최대 16개 LRU, 종료 파일/RAM 무결성,
함수 소유권 복구를 검사한다. 새 보충 출력은 캐시에서 제외한다. native QA·geometry·분할
정합·원본 pin 검사는 생략하지 않는다. helper 자체도 시작/재개/종료 코드 pin에 포함한다.

실제 overlay의 내용·JSON·deepcopy 호환 검사와 캐시 회귀 41개를 통과했다.
별도 20회 읽기는 원형 0.661초, 캐시 0.121초·종료 검사 포함 0.142초였다.
이는 해당 함수 단독 결과이지 5K 전체 검증·촬영의 개선율이 아니다.
기존 통합/분할/정적/strict/안전 회귀를 함께 확인했다. 이후 기본 경로가 기존 native 함수를
호출하지 않는지와 부모 계약 변조 거부·선택 불변성을 추가해 보충 55개·캐시 41개가 통과했다.
실제 native 승인·남은 부족분은 새 출력의 `status.json`·`selected_ledger.jsonl`을 확인한다.

`exact_v2`에서 첫 부족 slot `...00126`을 새 배치로 실제 취득하고 strict QA·안전 종료·
완료 증거 검증을 통과했다. **승인 4,942장·부족 58장**을 확인한 뒤 나머지를
tmux `base5-sunny15-deficit`에서 재개했다. 첫 시험의 정상 PAUSE는 `--stop-after-cells 1`의
의도된 경계다. 기존 4,941장·부모 status·ledger는 변경하지 않았다.

새 읽기 전용 웹 대시보드는 **http://localhost:8077/** 이다.
tmux `base5-sunny15-deficit-dashboard`에서 아래 명령으로 실행하며 기존 8075 서버는 유지했다.
실제 `/status.json` 응답이 `exact_v2`의 승인 4,942·부족 58을 제공하는 것을 확인했다.

```bash
python3 -B dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root artifacts/evaluation/base5_sunny15_completion_261018/exact_v2 \
  --manifest artifacts/evaluation/base5_sunny15_261017/finalization_v1/assembled_source_v2/final_source/manifest.json \
  --storage-root /home/luv/morai_cap/base5_sunny15_v159_261017 \
  --history-file artifacts/evaluation/base5_sunny15_completion_261018/dashboard_history.jsonl \
  --port 8077
```

수집 로그는 `exact_v2/console.log`다. 별도 서버는 관측만 하며 수집기·MORAI·PAUSE를 조작하지 않는다.

5,000장에 도달해도 학습 전달 준비 완료를 자동 선언하지 않는다. 새 selected/adopted manifest로
`finalize_base5_layout_split.py`의 실제 선택 기반 분할 검증을 다시 통과하고,
`qualify_base5_training_inputs.py`로 Q90/upright/Animal→Pedestrian 전달본을 검증한다.
기존 `finish.py finish`는 fresh-parent 전용 계약이므로 이 unified completion에 바로 사용하지 않는다.
Vast에는 검증된 `training3_jpeg_portable`만 전달하며 중간 PNG portable·JPEG cache를 함께 보내지 않는다.

<a id="flexible-completion"></a>

### 부족분 정책 전환: 같은 패밀리·학습 목적의 새 장면

`exact_v2`는 기존 자산 조합·객체 수·거리·방향 등을 함께 고정해 새 geometry를 만들었다.
본수집에서 실패한 어려운 조건만 남은 상황에서, 다른 geometry에서도 센서 근거가 부족하거나
새 배치 생성이 실패해 보충 효율이 낮았다. 11개 슬롯을 시도해 3개를 추가 승인한 시점에
사용자가 정확한 소수 분포 복원보다 남은 대회 준비 시간과 수집 효율을 우선하도록 요청했다.
현재 셀의 실제 종료·정지/P를 확인하고 `paused`로 멈췄다. 승인 **4,944장·부족 56장**과
원본·native GT·실패 기록·기존 실행 코드를 그대로 보존한다.

새 진입점은 `scripts/portable_base5/flexible_complete.py`다. 기존 4,944장의 native QA를
다시 실행하지 않는다. 같은 패밀리·미션·의미적 객체군·실제 동작 종류·capture split과
대략적인 거리 목적 안에서, 이미 승인된 후보의 구성을 참고해 **새 geometry**를 만든다.
정확한 primary 자산·보조 객체 목록·객체 수·도로 위치·방향은 바꿀 수 있다.
근거리 장애물과 80~100m 차량 목적은 보호하며, 60~80m 보조 차량만으로 80~100m 몫을
채우지 않는다. 원형 크기와 새 native QA·GT·지면·충돌·통신·정지/P 기준은 유지한다.
이전에 실패한 원본의 미관측 계획 quota를 새 선택에 강제하지 않지만, 실제 승인분은 삭제하지 않는다.

각 대체 슬롯의 원본 ID·donor·새 시나리오·정책·구현 SHA를 따로 기록한다. 새 동결 코드를
기존 `exact_v2`에 강제로 resume하지 않는다. `--prepare`는 CPU 후보 준비만 하며
`--run --confirmed-environment sunny15`만 실제 촬영을 허용한다. 슬롯별 시도 이력과 상한은 재개해도
초기화하지 않는다. 시간 예산은 각 실행의 추가 작업 시간이며, 안전 중단을 일반적인 부족분으로
우회하지 않는다.

첫 CPU 준비 `flexible_v1`은 native 0장 상태에서 donor의 파생 split 필드 누락을 검출해
중단했다. 원본·기존 승인분에는 영향이 없으며 당시 출력은 보존한다. 실제 소량 촬영 결과와
최종 운영 출력은 아래에 별도로 기록한다. 준비·테스트 통과를 촬영 완료로 표현하지 않는다.
`flexible_v2`에서는 CPU 후보 준비를 통과한 뒤 첫 native 장면이 9.778초에 승인·최종 P를
통과했다. 다만 외부 exact 검증기가 2회 시도 계약을 고정해 상위 합산 전에 중단했다.
이 출력과 native 시험 한 장은 보존한다. 새 전용 검증기는 실제 1회 계약·report·quota를
재구성하고 원형 direction/native/Park/identity 검사를 재사용한다. 저장된 시험 한 장의
읽기 전용 재검증과 기존 exact·캐시 포함 CPU 회귀 **123개**가 통과했다.

첫 연속 취득 출력은 `artifacts/evaluation/base5_sunny15_completion_261018/flexible_v3`다.
기존 4,944장을 그대로 계승하며, 앞선 시험 한 장은 중복 합산하지 않는다.
이 출력에서 7개 슬롯이 연속 승인돼 **4,951장·부족 49장**을 확보했다. 이후 01138의 CPU 후보가
목적 거리 경계를 벗어나 상위 실행이 중단됐다. donor는 원래 11.92m였지만 재생성 허용 범위가
10~30m여서 20m 경계를 넘을 수 있었다. 이 때 아직 새 native 촬영은 없었고, 앞선 7개의 native
승인·정지/P 근거는 보존됐다. v3의 `halted`와 `selected_not_certified`를 정상으로 고치지 않는다.

수정 실행 `flexible_v4`는 목적 거리 창에 완전히 들어오는 donor를 우선하며, 생성 중 경계를
넘거나 실제 layout이 겹치는 후보만 원형의 제한된 CPU 재생성 경로로 보낸다. 입력·해시·계보·
native·통신·Park 오류를 일반적인 후보 실패로 바꾸지 않는다. `--import-native-completion`은
진단된 CPU 중단 출력의 새 완료 child만 직접 검증해 계승한다. 기존 4,944장의 native QA는
재실행하지 않고, 새 승인 7장·소비한 시도 횟수·실패 payload 이력을 그대로 인계한다.
이 인계에서도 구 status·state·선택 기록을 수정하지 않는다. 관련 CPU 회귀는 총 **126개** 통과했다.

현재 운영 출력은 `artifacts/evaluation/base5_sunny15_completion_261018/flexible_v4`다.
tmux `base5-sunny15-flex`와 로컬 **http://localhost:8077/** 관측기를 사용한다.
기존 8075는 이전 본수집 출력이다. 새 콘솔은
`artifacts/evaluation/base5_sunny15_completion_261018/flexible_v4_console.log`다.

```bash
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311

# 실행 중에는 중복 실행하지 않는다. 정상 PAUSE 이후에는 같은 명령으로 이어간다.
python3 -B scripts/portable_base5/flexible_complete.py \
  --profile artifacts/evaluation/base5_unified_runtime_261018/sunny15_profile.json \
  --parent artifacts/evaluation/base5_sunny15_completion_261018/exact_v2 \
  --output artifacts/evaluation/base5_sunny15_completion_261018/flexible_v4 \
  --import-native-completion artifacts/evaluation/base5_sunny15_completion_261018/flexible_v3 \
  --run --confirmed-environment sunny15 --time-budget-sec 3600
```

기본 최대 3회이며 이미 완료한 슬롯·시도는 재개 때 다시 찍지 않는다. native 셀 안전 종료
후 PAUSE/STOP/시간 경계를 적용한다. `halted` 출력에 일반 재개를 강제하지 않는다.
5,000장 도달 후에는 같은 인자의 `--run --confirmed-environment sunny15` 대신 `--finalize`로
실제 선택 기반 layout을 확정한다. 실패한 원래 계획의 미관측 train floor만 제외하고
승인 prefix·실제 geometry·group·alias·누수·native 근거 검사는 유지한다.
이후 Q90/upright 학습 전달본 검증은 별도다. 원거리 AP나 새 모델의 성능 향상을 인증한 것이 아니다.

18:20경 첫 네 슬롯은 **4/4 승인**으로, 부모 4,944장에 새 4장을 더해 **4,948장·부족 52장**을
확인했다. 모두 실제 native 승인·최종 P·무접촉 근거를 통과했고 보충을 계속한다.
이는 초기 표본이며 전체 56장 승인율을 확정하거나 정확한 완료 시간을 보장하는 수치가 아니다.

| 부족 슬롯 | 기존 primary → 새 primary | actor 수 | 유지한 거리 목적 |
|---|---|---|---|
| 00211 | Lane_Separation_Bar → Bollard04 | 2 → 1 | 음영 중거리: 계획 32.59 → 42.04m |
| 00577 | WoodBox → CargoBox | 6 → 1 | 20~30m 장애물: 계획 28.53 → 24.04m |
| 00666 | Bollard01 → PE_Firewall_Orange | 4 → 1 | 0~20m 장애물: 계획 1.68 → 6.81m |
| 00693 | Porter2 → DEFAULT_SUV | 3 → 1 | 90~95m 보조 차량 조건 → 실제 primary 86.01m |
| 00727 | DEFAULT_MPV → Kia EV6 | 2 → 1 | 95~99m 보조 차량 조건 → 실제 primary 82.81m |

00693은 LiDAR 0점이지만 native 카메라 160픽셀·기존 320 crop 76픽셀의 양성 가시 근거가 있어
기존 QA가 승인했다. LiDAR가 없다는 이유로 버리거나, 반대로 카메라 근거가 없는 장면을
승인한 것이 아니다. 후보 생성 약 12초·정적 native 취득 약 9.6~9.8초 외에 새 증거 검증 비용이
남아 있다. 효율 개선의 핵심은 실패 조합의 정확한 복원을 고집하지 않아 승인 낭비를 줄인 점이다.
이후 여섯 번째까지 **6/6 승인·4,950장·부족 50장**을 확인했다. 00727도 LiDAR 4점과
카메라 원본 150픽셀의 양성 근거를 통과했다. 세부 95~99m 칸은 완화했지만 80~100m 목적은 유지했다.

v3 정상 취득 구간은 7장에 313.82초, 승인당 44.83초였고 실제 native 완료 간격은 평균 42.00초였다.
이 속도로 남은 49장을 선형 추정하면 36.61분이다. 재개 인계·일부 재시도를 포함한 운영 예상은
약 40~60분으로 설명했다. 코드 수정 중 정지 시간과 아직 발생하지 않은 어려운 장면 비용을
완료 속도로 숨기지 않는다. 전체 원격 5K의 ETA와는 다른 로컬 보충 추정이며 완료 보장이 아니다.

빌린 PC Sunny13은 확인 당시 검사 4,192장·승인 4,151장·부족 41장·미검사 808장으로,
약 30분간 239장(약 481장/h), 마지막 15분 약 455장/h를 승인하며 정상 본수집 중이었다.
로컬의 낮은 보충 효율과 다른 상태이므로 현재 본수집은 중단하거나 코드를 교체하지 않는다.
**원격에는 유연 보충을 아직 적용하지 않았다.** 정상 본수집 종료 후 남은 부족분에만 검증된
새 정책을 적용한다. 동결 bundle·runtime·profile·수집 원본은 그대로 보존한다.
마무리 시점에 한 번 더 읽기 전용으로 확인한 원격 상태는 `running`·승인 4,276장·부족 44장·
미검사 680장·오류 없음이었다. 부모 heartbeat는 확인 0.24초 전으로 최신이었고,
이전 확인보다 승인 125장이 늘었다. 원격 파일·프로세스 변경은 하지 않았다.

## 코드 경로

- 공통 생성: `scripts/portable_base5/generation.py`
- 고정 입력 경계: `scripts/portable_base5/session_input_boundary.py`
- 안전 PAUSE 인계: `scripts/portable_base5/unified_paused_import.py`
- 공통 수집: `scripts/portable_base5/unified_capture.py`
- 정상 종료 후 부족분: `scripts/portable_base5/complete_unified.py`
- 같은 패밀리·학습 목적의 유연 보충: `scripts/portable_base5/flexible_complete.py`
- 부족분 검증의 분할표 해석 재사용: `scripts/portable_base5/overlay_read_cache.py`
- 빌린 PC의 격리 실행: `scripts/portable_base5/unified_borrowed.py`

현재 유효 상태는 [CURRENT](BASE5_V2_CURRENT.md), 기존 준비와 과거 실패는
[fresh 환경 런북](261010_base5_fresh_environment_runbook.md)을 함께 참고한다.
