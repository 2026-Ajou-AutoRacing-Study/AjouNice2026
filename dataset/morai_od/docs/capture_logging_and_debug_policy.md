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

## Capture Monitor V2

Temporal V2 production 수집은 다음 read-only sidecar를 기본 monitor로 사용한다.

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/monitor_ros_capture_progress_v2.py \
  --run-dir /home/luv/morai_cap/<실제_실행_폴더> \
  --manifest \
  dataset/morai_od/manifests/temporal_sequence_v2_batch001_primary_runtime.json
```

한 번만 출력하거나 다른 로컬 도구에서 작은 snapshot을 읽으려면 다음 옵션을 쓴다.

```bash
python3 dataset/morai_od/scripts/monitor_ros_capture_progress_v2.py \
  --run-dir /home/luv/morai_cap/<실제_실행_폴더> \
  --manifest \
  dataset/morai_od/manifests/temporal_sequence_v2_batch001_primary_runtime.json \
  --once --no-clear --no-color

python3 dataset/morai_od/scripts/monitor_ros_capture_progress_v2.py \
  --run-dir /home/luv/morai_cap/<실제_실행_폴더> \
  --manifest \
  dataset/morai_od/manifests/temporal_sequence_v2_batch001_primary_runtime.json \
  --json
```

V2는 다음 정보를 표시한다.

- scene과 검증 frame 진행률, elapsed time과 ETA.
- mission dynamic, static temporal, hard transition 그룹별 quota.
- 13개 family와 train/val/test split별 완료 수.
- 최근 scene 처리시간 p50/p95와 실제 sequence rate p50/p05.
- checkpoint age, disk free space, 연속 실패 수와 실패 유형별 집계.
- 최근 실패 여섯 건의 ID, 유형과 짧게 자른 오류 내용.
- `console.log`가 있으면 현재 scenario load/capture 단계와 clip frame 진행률.

수집 영향 방지 원칙은 다음과 같다.

- capture runner 코드를 import하거나 수정하지 않는다.
- ROS topic을 구독하지 않는다.
- Camera/LiDAR sensor directory를 반복 scan하지 않는다.
- 2초마다 1 MiB 이하의 atomic `summary.json`만 다시 읽는다.
- `scenes.jsonl`과 `failures.jsonl`은 최초 한 번 읽고 이후 append된 byte만 읽는다.
  `console.log`는 최초에도 마지막 256 KiB만 읽고 이후 append만 추적한다.
- run directory에 파일을 쓰지 않는다. Monitor가 종료되거나 예외가 나도 capture
  runner에는 영향을 주지 않는다.
- `--interval-sec`은 최소 1초로 제한하고 production 기본은 2초로 유지한다.

실제 52-scene 결과에서 실패 유형 `actor_missing=6`과 높은 실패율 경고를 정확히
표시했다. 완료된 Temporal V2 clip에서는 8/8 frame, sequence 11.43 Hz와 scene
duration 16.15초를 복원했다. 동일 snapshot 1,000회 refresh benchmark는 평균 약
0.08 ms/refresh였고, production 기본 2초 주기로 환산한 monitor CPU 비율은 약
0.004%였다. 이 부하는 capture에 영향을 줄 수준이 아니다.

선택 실행은 manifest 전체가 아니라 실제 selector를 monitor에도 반복해 전달한다.
그렇지 않으면 family target이 전체 manifest 기준이 되어
`manifest_selection_mismatch` 경고가 발생한다.

```bash
python3 dataset/morai_od/scripts/monitor_ros_capture_progress_v2.py \
  --run-dir /home/luv/morai_cap/<smoke_실행_폴더> \
  --manifest \
  dataset/morai_od/manifests/temporal_sequence_v2_batch001_primary_runtime.json \
  --scenario drc001 --scenario dmc001
```

### 후순위 ROS diagnostics 계획

`/diagnostics`와 `rqt_runtime_monitor` 연동은 첫 production 수집 범위에서 제외한다.
여러 perception/tracking/prediction node를 동시에 운영하는 단계에서 다음 상태를
표준 ROS DiagnosticArray로 통합할 때 구현한다.

- ScenarioLoad, SaveSensorData, CtrlCmd 연결 상태.
- Ego/Object heartbeat age.
- capture checkpoint stale, disk 부족과 연속 실패 상태.
- detector, tracker와 prediction node의 heartbeat와 처리율.

Capture Monitor V2가 dataset 진행과 실패 문맥을 담당하고 ROS diagnostics는 전체 ROS
stack의 공통 health GUI를 담당한다. 두 기능을 중복 구현하지 않는다.
