# Base5 v151 전방 보강의 유한 보충과 재개 검증

문서 상태: EVIDENCE / BLOCKED SUN13. 기준일: 2026-10-15.
이 문서는 [앞선 70장 진단](261015_base5_front_far_compound_experiments.md)의 후속 작업이다.
완료 Sunny13 5K, v150 산출물, E24와 모델 설정은 보존하며 5K 본수집·학습은 실행하지 않는다.

후속 갱신: 같은 날 [v152 거리·3D 가림 보완](261015_base5_primary_obstacle_range.md)에서
작은 물리 primary의 거리 명세와 보충 후보 선택을 보완해 새 Sunny13 release를 동결했다.
아래 v151의 부족 1장·`bulk_ready=false`·비용은 당시 결과이며 수정하지 않는다. 현재 기준은 위 후속 문서와 CURRENT를 따른다.

## 변경 목적과 범위

v151은 추가 장애물 672개 중 자산별 약 절반인 338개를 전방으로 옮긴 배치 후보다.
초기 구현에서는 다른 seed의 donor를 가져오는 유연 보충과 동결된 전방 할당이 충돌했다.
실패분을 보충하면서 전방 배분이 사라지거나 자산 수가 바뀌지 않도록 별도 adapter를 연결한다.

| 입력 장면 | 보충 정책 | 보존하는 조건 |
|---|---|---|
| v151 전방 계획 있음, 선택·비선택 모두 | donor를 쓰지 않고 같은 source spec으로 새 geometry 탐색 | seed, 자산·actor ID·개수, 동작, split, 선택 여부와 전방 거리 band |
| 전방 계획 없음 | 기존 같은 family의 유한 donor 보충 | 기존 motion·split·희귀 primary 하한; 전방 계획이 있는 donor는 제외 |
| 이미 승인된 부모 장면 | 그대로 연결 | raw·선택 row·부모 journal 보존; 다시 촬영하거나 중복 합산하지 않음 |

새 geometry의 Ego·객체 위치는 달라질 수 있다. 고정 pose A/B나 원래 exact geometry quota의
보존을 주장하지 않는다. 전방 계획이 있는 source의 actor 정체성은 data ID뿐 아니라 unique ID,
name, source list, population pool까지 비교한다. 기존 front helper와 저장된 계획의 SHA는 바꾸지 않는다.

보충은 기존 최대 3회·호출별 1,800초 경로를 유지하며 허용 인자 상한은 5회다.
각 전방 후보는 최대 512개 geometry proposal 중 먼저 유효한 8개를 평가한다.
이미 사용한 scenario SHA를 제외하고, 재개해도 durable attempt 번호를 초기화하지 않는다.
이 탐색은 CPU 배치 준비이며 native 센서 QA 통과를 대신하지 않는다.

## 검증 설계

결과를 보기 전에 full 5K와 기존 회귀 목록에서 64+8장을 등록했다.

| 표본 | 개수 | 확인 목적 |
|---|---:|---|
| 19종 추가 장애물의 전방 선택 / 기존 비선택 | 38 | 각 자산의 두 stratum과 실제 native QA |
| 기존 긴 정지 NPC 보완 전체 | 24 | 원래 수정 payload에서 실제 정지와 도로 지지 관련 회귀 |
| 과거 부족분 장면 / 고정 정적 대조 | 2 | 과거 실패를 강제로 재현하지 않고 현재 결과 관측 |
| 실제 이동 회귀 | 8 | 이동·두 waypoint·심각한 부유·안전 검사 유지 |
| 계획된 대체 경로 probe | 4 | 선택/비선택 × 작은/큰 footprint에서 새 geometry와 보존 계약 |

네 probe는 **고의로 실패시킨 부모의 보충이 아니다**. 미리 등록한 별도 진단용 장면이며
기존 5K·train/val·실제 부족분 수에 합산하지 않는다. 부모64에서 실제 부족분이 없다면
실패 row를 만들거나 성공분을 빼서 보충 검증을 통과시키지 않는다.

첫 16장 뒤 정상 일시정지하여 selected row와 raw의 SHA·inode·mtime·size를 저장하고,
나머지를 재개한 뒤 기존 16장의 불변성을 비교한다. 완료 후 같은 명령을 다시 실행해
추가 native 촬영 0과 선택·raw·누적시간 불변을 확인한다. 안전 종료 Park가 불확실한 세션은 재개하지 않는다.

## 동결과 실행 보호

새 release는 현재 runtime의 로컬 import·실제 adapter 파일과 full 5K의 source/metadata,
frozen specs, 센서·네트워크·MGeo·자산·경로·template 및 native 검증 근거를 SHA로 연결한다.
코드만 같고 외부 입력이 바뀐 경우도 launcher 사전 검사에서 거부한다.
이 전수 검사는 실행·재개 시 사전 검사이며 매 프레임에 반복하는 연산이 아니다.

실제 접촉·GT·가시성·이동·Ego 정지·Park 기준은 완화하지 않는다.
일반 품질 부족분만 유한 보충으로 넘기고 통신·CollisionData 무수신·방향 불일치·hash 불일치는 안전 중단한다.
과거 v150의 `bulk_ready=true`를 현재 source의 검증 완료로 재사용하지 않는다.

## 정적 실측과 남은 사례

첫 수집은 64장 중 62장 승인·2장 부족분이었다. 유한 보충은 4개 새 payload를 실제 촬영했고,
1장을 보충하여 최종 63/64가 됐다. 성공 62장과 실패 원본은 보존했다. 첫 16장의 raw SHA·inode·mtime·size,
선택 row와 기존 제어 이벤트 로그는 나머지를 재개한 뒤에도 동일했다.

남은 `sunny13_sunny11_b5v2_01968`은 전방 보강을 받지 않은 legacy stratum이다.
추가 actor는 Scooter_01이지만 실패한 **주 객체는 Lane_Separation_Bar**다.
주 객체의 계획 거리는 원본 88.46m이며 새 배치 세 번은 66.96 / 78.61 / 74.21m였다.
각각 LiDAR 0점, 전방 원본 instance 7 / 3 / 6px였고 세 번째는 측방에도 1px뿐이었다.
세 번 모두 `primary_no_sensor_evidence`로 남았다. Z 오류나 충돌 때문에 실패한 사례가 아니다.
실패 row를 승인으로 고치거나 더 가까운 물체의 점을 주 객체의 근거로 사용하지 않는다.
현재 고정된 거리·자산 계약으로 무한 반복하는 대신, 후속 분포 수정 여부를 별도로 판단해야 한다.

최종 정적 63장의 parent 248개 중 지면 평면으로 정상 gap을 확인한 것은 154개다
(−7.58~+10.50cm). 93개는 평면 미측정 경고이고 1개는 +16.87cm 경고다.
마지막 경고는 기존 정적 advisory 정책의 심각한 오류 기준 ±40cm 이내라 유지됐다.
이는 mesh 접지 인증이 아니며, 경고 94개를 심각한 부유 94개로 해석하지 않는다.
승인본의 Ego Z span 최대는 1.29mm였다. 원래 보완한 긴 정지 NPC 24장은 모두
최종 계획 payload 그대로 승인됐고 실제 정지 witness를 만족했다.

실제 이동 회귀는 8/8 승인이다. Woman8·Girl1·Woman3·Boy2의 사거리와
Dog1·Dog2·Ped_Pet1·Woman2의 음영 이동을 포함하며 기존 native 이동·높이·안전 기준을 유지했다.
관측된 최대 단일 Z step은 약 0.81mm였고 심각한 Z 점프는 검출되지 않았다.
이는 관측 구간의 결과이며 모든 경로의 mesh 접지나 임의 환경의 이동을 인증하지 않는다.

## 최종 판정

| 검증 항목 | 결과 |
|---|---|
| 정적 64장 | 최초 62 승인 → 실제 부족분 2장에 새 payload 4개 → 최종 63 승인·부족 1 |
| 19종 × 선택/비선택 native stratum | 37/38 승인. 남은 1개는 위 원거리 주 객체 때문 |
| 기존 긴 정지 NPC | 24/24, 최종 원래 payload·실제 정지 witness 확인 |
| 실제 이동 | 8/8 승인 |
| 별도 대체 경로 probe | 4/4 승인. 기존 672 / 전방 338 / 비선택 334 / 19종 할당 보존 |
| 첫 16장 이후 재개 | 기존 16장 선택·원본·제어 이벤트 보존 |
| 완료 후 재실행 | 부모·보충·probe 추가 촬영 0, 원본·선택·제어 이벤트·누적시간 불변 |
| 최종 CPU 회귀 | 774개 통과, `git diff --check` 통과 |
| 새 release | `completion_live_qualified=true`, `front_recovery_live_qualified=true`, **`bulk_ready=false`** |

완료 후 재실행에서 부모 status의 `updated_at`만 갱신됐다. 처음에 status 전체 SHA까지
불변으로 요구한 진단 비교는 이 관측 시각 갱신 때문에 실패했다. 그 원본 비교도 보존하고,
두 번째 비교에서 실제 변경 필드를 확인했다. raw·선택·이벤트·누적시간·completion state는 그대로였으며
상태 표시 시각 갱신을 새 촬영으로 세지 않는다. 이 차이는 별도 receipt에 기록했다.

새 release의 runtime 73개와 source/input/native evidence 15,969개를 동결했다.
실제 launcher 사전 검사는 통과했고, `bulk_ready=false`인 release로 시작하려는 음성 대조는
native driver 호출 0회로 거부됐다. 이를 우회하는 gate 변경이나 5K 시작은 하지 않았다.
마지막 네 probe의 종료 Park는 모두 확인됐고 종료 후 Ego 속도도 약 0m/s였다.

현재 64+8 표본에서 QA·실패 비용을 포함한 5K 가중 예상은 **9.75시간 / 36.05GiB**다.
운영 예산 8~12시간 / 45GiB는 유지한다. 이전 v150의 8.14시간과 표본·실패 비용이 다르므로
이를 코드 속도 퇴행의 직접 증거로 해석하지 않는다. 본수집 실측이나 5K 성공률 인증도 아니다.

다음은 낮고 작은 원거리 primary에 대해 자산별 거리 분포 또는 명시적인 최종 대체 정책을 결정하는 것이다.
이번 한 ID만 임의로 예외 처리하거나 무한 재시도하지 않는다. 정책을 바꾸면 새 후보·새 감사·영향받는
소량 회귀로 확인한 뒤 새 release를 동결한다. 원거리 Vehicle quota와 복합 component GT는 이번에 바꾸지 않았다.

## 산출물

작업 root는 `artifacts/evaluation/base5_generator_recovery_261015`다. raw와 실행 journal은 Git에 넣지 않는다.

| 산출물 | 의미 |
|---|---|
| `registration_v151/registration.json` | 결과 관측 전 등록한 정적64·이동8과 full source hash |
| `qualification_static64` | 부분 재개를 포함한 현재 runtime 정적 회귀 |
| `partial_resume_before.json` | 첫 16장 완료 시점의 원본·선택 snapshot |
| `qualification_static64_completed` | 기존 62장 + 새 승인 1장, 부족 1장과 네 시도 모두 보존 |
| `qualification_animated8` | 실제 이동 8장 |
| `front_recovery4_v4` | 실제 4장·완료 재실행 0장 proof. v1~v3는 CPU-only 준비 이력 |
| `partial_resume.json`, `partial_resume_after.json` | 첫 16장과 최종 부모 62장 사이의 보존 증거 |
| `terminal_resume.json`, `terminal_resume_{before,after}_v2.json` | 부모·보충의 raw/선택/시간 불변 근거 |
| `terminal_status_refresh_v2.json` | 부모 `updated_at`만 갱신된 원문 대조 |
| `release_v151_sunny13/release.json`, `cost_estimate.json` | 현재 source의 차단 상태·해시·비용 근거 |
| `release_v151_sunny13/launcher_guard_check.json` | 사전 검사 통과·차단된 시작 거부·native 호출 0 |

full 5K 후보는 앞선 실험 root의 `candidate_v151/manifest.json`이며 SHA는
`47c8af42d6c38dfb10a54b8ff27ff86c37f6e0b2c6fed79b56d8f2f6390b0332`다.
완료 Sunny13 canonical ledger SHA는 기존 `026e6244…a43` 그대로다.
새 source의 동결과 보존된 진단을 완료했지만 본수집 승인을 뜻하지 않는다.
