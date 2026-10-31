# Base5 v41 — guarded bridge live·중복 촬영 제거

2026-10-06. EVIDENCE. 본10K 캡처·학습 미시작. Production 변경 없음.

## 결과

사용자 bridge 재시작/MR 재연결 후 guarded bridge에서 Park12회가 모두 통과했다.
간격0/50/250ms 각각4회, 응답약3.3–18.5ms다.
남은267/406/925/1028 네셀을 새 output에서 보완까지4/4 통과했다.
기존v39/v40의 안전하게 선택된28셀과 합쳐 대표32/32셀이다.
이는 단일 guarded 세션에서32셀 전체를 다시 촬영했다는 뜻은 아니다.
UUID 세대 요청ID와 MORAI 응답의 live 호환성을 확인했다.

## 중복 촬영 제거와 실측

기존 batch는 review가 나오면 단일셀 보완 루프에서 같은 원본을 다시 찍었다.
새 경로는 이미 촬영된 안전한 review를 검증하여 보완의 초기 증거로 사용한다.
원본을 이동하거나 audit/GT를 바꾸지 않고, 보완된 시나리오는 반드시 다시 캡처한다.

| 같은 원본4셀 | 기존 보완 | review 재사용 보완 |
| --- | ---: | ---: |
| 외부 wall time | 53.93초 | 41.95초 |
| 최종 quota | 4/4 | 4/4 |
| 실제 raw 촬영 | 6회 | 5회 |
| EventCmd 호출 | 14회 | 14회 |
| 5초 timeout/미종료worker | 0/0 | 0/0 |

wall time 약22.21% 절감이다. 보완이 필요한 동물 한셀의 중복 촬영을 없앴다.
보완 시 입력Z는 매번 실제 지면 측정값으로 계산하므로 최종925 payload SHA는 다르다.
두 실행의 shift는 각각 약−0.213678/−0.214384m이며 양쪽 최종 audit를 통과했다.
따라서 입력 원본4셀의 대응 비교이며 최종 렌더/보완 payload가 비트단위 동일한 A/B는 아니다.
순차 실행1쌍·보완 비중이 높은 소량 표본이다. 10K 전체22% 절감이나 완료시간을 보장하지 않는다.

새 실행에는 reload 중 기존 handler 종료로 `service handler returned None` 오류2회가
각각0.214/0.215초에 반환됐다. 기존 bounded retry로 정상 응답했고 모든 boundary/final
Park를 확인했다. 무오류는 아니며 긴 timeout 대신 짧은 전환 오류가 남는다.
기존 실행14회는 모두 첫 요청 정상 응답했다.

## 안전·resume·변환

재사용 전 manifest/code/config/raw provenance를 `prepare_session`으로 검증한다.
같은 원본cell/payload·review상태·모든boundary/finalPark·원본audit일치가 필수다.
calibration/물리충돌/실패session은 재사용하지 않는다. seed SHA도 campaign계약에 포함한다.
충돌/Z/지면/가시성 gate·scene별network reload·Park조건을 완화하지 않았다.

Base5 단위검증123 tests PASS. 새완료campaign `--resume`가 추가촬영 없이 통과했다.
CPU `lidar1_camera0`·ROI[-100,-30,-3,100,30,2] 변환은4장/GT7개,
train2/val2/test0(vehicle3/pedestrian2/animal2), review/failed 편입0이다.
본 모델학습이나 새로운 모델성능 평가가 아니다.

## 근거 경로

공통 root `/home/luv/morai_cap/base5_v2_261006/` 아래:

- `park_sequence_guard_v41.json`
- `batch_last4_guard_v41`: 기존중복 초기촬영 포함 baseline, real53.93초
- `batch_last4_seed_guard_v41`: seeded recovery, real41.95초
- 각 campaign의 `selected_ledger.jsonl`, `chunk_*/capture/event_cmd_requests.jsonl`,
  `shutdown_safety.json`, recovery `report.json` 및 height_deck metadata
- `seed_guard_v41_infos`: 선택4장 CPU변환

완료resume는 status의 elapsed를 재계산하므로 최초실행 wall time은 위 기록을 사용한다.
코드hash 변경 이전 baseline/실패session은 현코드로 강제resume하지 않는다.
대시보드 http://127.0.0.1:8077 은 새 seeded campaign을 가리킨다.

## 다음

대표32셀 coverage는 완료했다. 다음은 family별 보완 빈도를 포함한 더 넓은 bounded
pilot에서 throughput/서비스 전환/중단·resume를 확인하고 비용을 재추정하는 단계다.
정적셀의 pose/control 준비와 동적셀의 target 도착 대기가 남은 주요 비용이다.
대기시간을 무작정 줄이거나 움직임 증거를 생략하지 않는다.
10K 본 수집 driver 승인·장시간 안정성 인증은 아직 아니다.
