# Base5 정지 검사 중단 원인과 성공분 보존 재개

2026-10-09 / 수집 runtime v133, 입력 v132 Sunny13 5K 유지.
캡처 직전 Ego 정지 검사 실패를 통신 실패와 분리했다. 기존 성공분을 원래 runtime으로 검증해
새 output에서 참조하며, 불안정한 장면을 승인하거나 전체 데이터를 다시 수집하지 않는다.

## 중단 원인

2026-10-09 14:20:47 KST, 원래 `collection_5k_final_v132_sunny13`이
`Unsafe/non-budget target failure`로 중단됐다. 실제 선행 오류는 chunk 0477의
`b5v2_01981`: `Ego stationary witness lost before capture; no capture`이다.
Universe 버스 옆을 지나는 정적 근접 회피 장면으로, 계획 BBox 경계 간격은 약 0.375m다.

캡처 직전 약 0.383초/20개 Ego 수신 샘플에서 최대 수평속도 0.182m/s,
수직속도 0.342m/s가 관측됐다. XY 변화 0.0086m/Z 변화 0.0091m는 작지만,
정지 조건은 위치 변화와 속도를 모두 검사하므로 속도 0.02m/s 조건을 초과했다.
SaveSensorData 요청 전 거부된 것이며, raw 파일 쓰기나 GPU 추론이 실패한 것은 아니다.

기존 수집기는 정지 witness를 한 번만 검사해 즉시 RuntimeError를 내고,
부모 배치 검사기는 해당 실패를 동적 목표 도착 실패로 검증하려 했다.
정지 실패에는 그 증거 형식이 없으므로 안전하지 않은 오류로 판단해 전체 중단했다.
모든 오류에 무조건 resume을 허용하지 않은 정책 자체가 아니라, 실패 분류가 부족했던 문제다.
boundary/final Park는 확인됐고 통신 단절 근거는 없다.

물리적인 순간 흔들림의 세부 원인은 확정하지 않았다. 같은 payload3회와 버스를 제거한
control3회에서 캡처 직전 실패는 다시 나타나지 않았다. 원래 장면은 일부 반복에서
actor 누락·정지 차량 위치 이동·지면 근거 부족 review가 발생했다.
근접 버스 접촉, 지형·차체 settling 중 어느 하나라고 단정하지 않으며 원본 장면은 부족분으로 보존한다.

## 수정된 처리

- 처음 정지 witness가 합격하면 추가 대기 없이 진행한다. 정적 장면에서만 실패 시 최대 2초
  같은 정지 조건으로 재확인한다. 허용 속도·위치·Z 기준은 바꾸지 않는다.
- 실제 이동 actor가 하나라도 있으면 재확인 대기를 추가하지 않는다. 도착 시점의 외관·거리
  분포를 바꾸지 않으며 불안정한 Ego 장면은 촬영하지 않는다.
- 재확인 뒤 Ego 위치·드리프트·CollisionData·객체 snapshot을 다시 읽는다.
  기다리기 전의 pose나 무접촉 증거를 재사용해 캡처하지 않는다.
- 지속 정지 실패는 해당 장면만 부족분으로 남긴다. fresh Ego/Object/CollisionData,
  해당 시나리오의 collision epoch, 관측된 무접촉, brake 유지, Park 응답과
  boundary/final Park가 증명된 경우에만 다음 장면으로 넘어간다.
- stale topic, 제어/Park 실패, collision epoch 미확인, 증거 checksum 오류는 여전히 전체 중단이다.
  실제 접촉은 기존 별도 collision 부족분 정책을 유지한다.
- 같은 실패 payload를 즉시 무한 재로드하지 않는다. 완료 후 `--resume --replenish`에서
  같은 자산·family·거리·방향·group/split의 새 배치로 제한된 보충을 수행한다.

## 성공분 복구

기존 output을 바꾸지 않고 저장된 `runtime_v132_source_reference.tar.zst`의 원래 코드로
batch/scene/recovery 계약, raw 파일 완결성, annotation/audit, Park를 검증했다.
code hash를 무시하거나 기존 session 계약을 새 hash로 덮어쓰지 않았다.
`base5_campaign_import.py`는 검증한 성공분의 경로와 증거를 `imported_prefix.json`에 기록한다.
새 runtime은 그 참조를 검증하며 이미 처리된 셀을 건너뛴다.

| 시점 | 승인 후보 | 부족분 | 미처리 |
|---|---:|---:|---:|
| 원래 중단 집계 |1885|23|3092|
| 원래 raw 재검증 후 |1887|24|3089|
| 새 output 실제 1셀 재개 후 일시정지 |1888|24|3088|

중단 직전 부모 집계에 들어가지 않은 01979/01980 성공 2장을 복구했다. 실패 01981은 승인하지 않았다.
새 root의 첫 남은 01982는 실제 캡처 후 승인됐다. 5K 전체를 다시 시작한 것이 아니다.

입력 manifest SHA256:
`89b053e7332e20f284c2de28d2f2365a3a9cf157d017c628481d9715ec5a1a77`.

새 output:
`~/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_final_v132_sunny13_recovered_v133`.
원래 output은 성공 raw의 참조 대상이므로 삭제하거나 이름을 바꾸지 않는다.
raw는 원래 validator의 완결성 검사와 크기/mtime 불변을 확인했다. 모든 raw 바이트를 새 SHA256으로
전수 읽은 검사는 아니다. manifest·scenario·metadata·audit·telemetry·계약 증거는 hash로 검증한다.

## 검증

- 실제 MR 정상 배치 8/8 승인. 원래 높이·정지·가시성·GT 조건 유지.
- 실제 음영 이동 2/2, 사거리 이동 2/2 승인. 이동 캡처에는 추가 정지 재확인 대기가 없음을 telemetry로 확인했다.
- calibration-only 장면의 witness 실패를 강제로 주입한 실제 MR 시험:
  2초 재확인 후 실패 1개, 해당 raw 요청 0회, 부모 campaign은 중단 없이 tail1개 승인으로 완료.
  테스트 주입은 별도 `run_base5_stationary_fault_probe.py`에서만 수행한다.
- 위 실패+tail의 완료 resume에서 추가 ROS capture 없이 같은 결과를 재검증했다.
- 실제 5K 새 output에서 이미 처리된 1911셀을 건너뛰고 다음 1셀만 취득해 일시정지했다.
  같은 지점의 재개를 ROS master 없는 주소로 반복해 추가 촬영 0·동일 1912셀 집계를 확인했다.
- CPU 단위 검사 422개 PASS. 증거 누락/변조·stale/Park·접촉 경계,
  재확인 성공/timeout/동적 무대기, import 중복 셀 방지, 준비 모드의 ROS 부작용 0을 검사했다.
- 정상 정적 8장 CPU 학습본 변환 성공: train 7/val 1, GT 12개, review/failed 전달 0.
  음영 이동 2장(GT 2개)과 사거리 이동 2장(GT 2개)도 CPU 변환했다.
- 대시보드는 새 root의 진행률과 원래 root의 저장량을 함께 계산한다.
  기존 6 환경 suite의 Sunny13 상태만 새 root로 연결해 중복 합산하지 않는다.

실제 5K 전수 성공, 순간 흔들림의 완전 제거, 최종 학습 성능을 보장하는 결과는 아니다.
122종·거리·yaw·5K/6 환경 30K·native GT·production 모델·planning/localization 계약은 미변경이다.

## 다시 수집하기

master와 guarded bridge는 현재 살아 있다. MR Capture Mode/Sunny13을 유지하고
[런북](261007_base5_5k_capture_runbook.md)의 터미널 3 명령만 실행하면 남은 셀을 이어간다.
대시보드는 `http://127.0.0.1:8075`이며 새 root로 전환했다.
코드/input/config를 다시 바꾸지 않고 동일 명령으로 안전한 중간 pause/resume을 사용한다.
통신/Park 불명확으로 halted가 되면 무조건 resume하지 말고 원인을 확인한다.
일반 실행이 완료되면 같은 명령에 `--replenish`를 추가해 부족분만 새 배치로 보충한다.

검증 자료는 `~/morai_cap/base5_stationary_v133/`에 보존한다. 본 수집을 계속 켜 두지는 않았다.
새 runtime 원본은 release의 `runtime_v133_recovery_source_reference.tar.zst`에 동결했다.
SHA256: `c28ece7f8391fa3c25a519cd7c895819302b42af259428b4a062de4435369907`.
