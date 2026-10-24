# Base5 v31 — 근접·혼합 마무리 / quota·resume 연결

2026-10-06. 소량 MR live 및 CPU wiring 검증이다. 본10K 수집·학습·GPU 추론은 하지 않았다.

## 남은 세 조건

`/home/luv/morai_cap/base5_v2_261006/quality_remaining_v31/report.json`:

- Deer1 이동 + Concrete_Barricade02: 최초 지면 미확인 → 빈 지형 촬영 → 독립 지면 reference를 적용한 재촬영 통과.
- BabyStroller3 두 개 정지: 최초 촬영 통과.
- Scooter1 + Niro 정지 혼합: 최초에는 actor/annotation/센서 근거 누락 → 같은 quota 셀 재배치·재촬영 통과.
- 객체5회 + 빈 지형1회, 최종 원래 셀3개 각각 한 장 선택. 모든 child 종료 Park 확인.
- 최초 실패 이유를 보존한다. 스쿠터 혼합이 첫 촬영부터 성공했다고 해석하지 않는다.

## 대량 경로 구현

`run_base5_capture_campaign.py`가 기존 bounded quality loop를 셀 단위로 호출한다.

- 원래 manifest의 셀 하나가 quota 하나다. 빈 지형 촬영·재촬영·복수 component GT는 quota를 늘리지 않는다.
- 최초 캡처 → 가능한 경우 입력Z/독립 지면 보완 → 같은 셀 재배치의 순서다. 기본 quality round당 재시도2회, quota round2개(허용1–3개)로 유한하다.
- 재배치는 primary asset, family/spec, mission, motion request, role, 거리 셀, split을 유지한다. 관측으로 입력을 보완한 후 새 센서/GT를 촬영한다. raw GT를 사후 이동해 통과시키지 않는다.
- 안전 종료된 target-arrival 실패는 재배치 후보다. 통신 실패, 물리 접촉, 미확인 Park은 후속 로드를 중단한다. 근거 없는 지형은 gate 면제가 아니라 재배치/부족 셀로 처리한다.
- 소진된 셀은 `quota.json` 및 `deficit_manifest.json`에 원래 셀 그대로 남는다. 같은 캠페인의 resume은 소진된 셀을 무한 재시도하지 않는다. 추가 보충은 deficit manifest로 새 bounded campaign을 명시적으로 시작한다.
- manifest·원본 scenario/metadata·네트워크/센서·관련 코드 hash 계약이 달라지면 resume을 거부한다.
- 완료 child는 raw provenance/센서 완결/session 계약을 offline 검사하고 재사용한다. 진행 도중 parent가 끊겼더라도 child의 안전 종료가 확인되면 동일 단계부터 재사용한다. shutdown 근거가 없거나 Park 실패인 child는 자동 재개하지 않는다.
- 원래 셀별 durable report가 authoritative checkpoint다. canonical `selected_ledger.jsonl`은 여기서 재구성한다. 통과 셀 중복 촬영/중복 집계를 막는다. 실패 raw는 삭제하지 않는다.
- campaign STOP은 실행 child로 전파하며 안전 종료를 기다린다. dashboard status는 child 대기 중에도2초 주기로 갱신한다.
- 64셀 초과는 `--approve-bulk-capture`를 요구한다. 플래그 존재가 본 수집 승인/장시간 인증을 뜻하지 않는다.

## 검증 범위

- 첫 live campaign `campaign_resume_v31`: 유모차/스쿠터 혼합2셀, 첫 셀 후 중단 → resume →2셀 통과. 완료 셀은 다시 로드하지 않았다.
- 최종 코드 live campaign `campaign_final_v31`: Deer+바리케이드 및 유모차2셀에서 보완 경로와 첫 셀 후 중단/resume 검증. 상세 수치/상태는 같은 폴더의 quota/status/child report가 근거다.
- 최종2/2셀 통과, 부족0. 완료 후 read-only resume도 확인했다. `campaign_final_infos_v31` CPU 변환 train2장/GT7개(animal1·obstacle3·pedestrian3), evidence policy로pedestrian component1개 제외, review/failed 장면 유입0이다. component 제외 근거/원본은 보존되며 quota는 프레임 기준이다.
- 테스트89개 PASS: 기존82개 + campaign7개. pause/resume 중복방지, unsafe child 후속 차단, 부족 manifest, 동일 원래 셀 quota 보충, 계약 변경/무승인 bulk 거부, STOP pending 보존을 포함한다.
- 실제10K campaign·장시간 서비스 안정성·모든 지형에서의 보완 성공률은 아직 검증하지 않았다. sparse-ground/unsupported route는 안전한 실패와 quota 보충으로 남을 수 있다.

## 다음

소량 선택 ledger의 학습 변환 CPU smoke까지 확인했다. 당시 다음이던 MR OFF 속도/메모리 비교는
[v32 EVIDENCE](../../../docs/bevfusion/261006_BASE5_BATCH1_CAPACITY_EVIDENCE.md)에서 완료했다.
본10K 시작은 별도 승인 이후다.
