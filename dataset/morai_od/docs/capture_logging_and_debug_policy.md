# MORAI 대규모 수집 로그·디버깅 정책

## 저장 파일

- `summary.json`: 진행률, 완료·실패 수, 검증 프레임 수, 완료/실패 scene ID
- `scenes.jsonl`: scene당 한 줄의 압축 결과
- `failures.jsonl`: 실패 scene만 상세 원인과 실제 actor 정보
- `console.log`: 장면 검증·완료, WARN, ERROR만 저장

성공 frame의 전체 파일 경로 목록과 전체 actor 관측값은 report에 반복 저장하지
않는다. 실제 데이터 파일명으로 복원 가능한 정보는 report에서 제외한다.

## Codex 및 사람의 확인 순서

대규모 실행은 다음 순서를 지킨다.

1. `summary.json`만 먼저 읽는다.
2. 특정 실패 원인이 필요할 때 `failures.jsonl`에서 해당 scene 줄만 읽는다.
3. 작은 smoke test는 필요하면 `scenes.jsonl` 전체를 읽어도 된다.
4. 수백 scene의 `scenes.jsonl`, `failures.jsonl`, `console.log` 전체를 한 번에
   모델 입력으로 전달하지 않는다.
5. 로그가 필요하면 마지막 일부 또는 특정 scene ID 검색 결과만 사용한다.

## 중단 후 재개

`summary.json`의 `completed_scene_ids`를 기준으로 성공 scene은 보존하고 실패·미실행
scene만 다시 실행한다. 재시도 결과는 기존 `scenes.jsonl`과 `failures.jsonl`에
append되며 `summary.json`은 작은 최신 상태 파일로 원자적으로 교체된다.

## 구형 대형 report 복구

`recover_capture_summary_from_files.py`는 구형 `ros_capture_poc_report.json`과
`console.log`를 열지 않는다. 각 sensor 폴더의 검증된 파일명만 검사하여
`summary.json`, `scenes.jsonl`, `failures.jsonl`을 복구한다. 구형 report를
통째로 읽지 않고 재개 목록을 만드는 용도다.

## 구형 console 실패 분석

구형 `console.log`의 실패를 다시 분석할 때도 원문 전체를 모델 입력으로 전달하지
않는다.

- `analyze_capture_console_stream.py`: 로그를 한 줄씩 처리하여 실패 유형과 actor
  종류별 누락 합계만 만든다.
- `analyze_capture_failures_against_scenarios.py`: 로그를 한 줄씩 처리하고 생성된
  scenario JSON과 대조하여 누락 차량 asset, 거리·방향, scene 유형, 실행 순서 구간별
  빈도를 작은 JSON으로 만든다.

두 스크립트의 작은 결과 JSON만 Codex가 읽는다. 로컬 스크립트가 대형 로그를
순차적으로 읽는 것은 허용하지만, 대형 원문을 채팅 context로 불러오지는 않는다.
