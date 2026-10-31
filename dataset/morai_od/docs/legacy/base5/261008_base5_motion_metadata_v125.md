# Base5 객체 이동 진단 메타데이터 — runtime v125

2026-10-08 13:05 KST. 사용자 요청은 동기 속도 GT 개발이 아니라, 리소스가 작다면
정적/동적 촬영 여부를 나중에 확인할 참고 정보를 보존하는 것이다.
현재 Base5-only/single-frame/4class/7D/velocity off와 rosbag 속도 평가 계약을 유지한다.
Production/모델/threshold/GT/배치/구성/quota/환경 입력을 변경하지 않았다.

## 기록 계약

`run_base5_capture.py`의 기존 Object callback/위치 trace를 확장했다.
성공적으로 보존한 캡처의 `telemetry.json`은 schema3이며 기존 키를 그대로 유지한다.

- `metadata.runtime_actors[].motion`: 요청한 static/standing/walking/running 등의 상태다.
  실제 움직임을 증명하는 태그가 아니다.
- `telemetry.motion_capture_actors`: 기존 endpoint/실제 위치 변화 검사를 유지한다.
  이 receive-time 미분값도 정밀 센서 시각 속도 GT라고 주장하지 않는다.
- `telemetry.object_position_samples`: 기존 `t/uid/x/y/z`에 `velocity`, `heading`,
  `source_seq`, `source_stamp`, `source_stamp_ros={secs,nsecs}`를 추가한다.
  exact ROS stamp를 따로 보존해 float 변환의 정밀도 손실을 원본으로 대체하지 않는다.
- velocity/heading은 `/Object_topic` 메시지 원문이다. m/s·월드 속도·BBox-center 속도라고
  자동 해석하지 않는다. 단위/좌표계/heading/회전 offset 보정은 적용하지 않는다.
- `t`는 수신 host monotonic 시간이며 `capture_requested_at_monotonic`과 같은 clock이다.
  `source_stamp`는 별도 ROS message clock이다. 둘을 직접 빼거나 센서 동기 시각으로 취급하지 않는다.
- `object_motion_sample_contract`에 위 의미와 한계를 함께 저장한다.
  필드 미제공은 null이며 속도0을 만들어 넣지 않는다. 실제 메시지0도 이동 없음의 유일한 증거로 쓰지 않는다.

추가 subscriber/서비스/SaveSensorData 요청/대기/보간/목표 속도 대입은 없다.
기존 shared actor trace의10000행 초과→마지막5000행 유지 상한을 그대로 사용한다.
장면 load 이후 trace만 보존하며 모든 준비 구간의 무손실 연속 기록을 보장하지 않는다.
캡처 전후 전체 Object snapshot도 그대로 보존한다. native 센서/GT는 변경하지 않는다.
기존 schema2 raw는 재작성하지 않는다. 실패용 telemetry가 새 성공 schema3와 항상 같은 필드 집합이라는
가정도 하지 않는다. 캡처 전 실패는 센서 파일/속도 근거가 없을 수 있다.

## 실제 MR 및 CPU 검증

새 진단 root: `/home/luv/morai_cap/base5_v2_261008/motion_metadata_v125_live`.
v123 `pilot_manifest.json`의 index0/25, 즉 `b5v2_00163`/`b5v2_00223`을
현재 runtime으로 새 output에서 실행했다. 구 pilot/본5K 입력은 보존한다.

| 확인 | 결과 |
|---|---|
| 실제 캡처 | 정지 G90 1 + Girl1 달리기 1, 승인2/2, 부족0 |
| 전체 pilot wall |19.882초; 기록 추가만의 latency 인과 대조가 아님 |
| 안전 | 두 child의 종료 Park 확인, 기존 움직임/Z/CollisionData gate 유지 |
| trace | 정지158행/달리기320행, schema3 필드 저장 확인 |
| 달리기 witness | 요청 전 실제 변위 속도≈2.230m/s, 원문 velocity.x≈7.920 유지 |
| 기록 크기 | 위치-only trace 대비 JSON 증가48703/96813B(같은 indent2 비교) |
| helper microbenchmark |8actor 메시지×10000회, 평균약5.52µs/호출; ROS 전체 latency 측정은 아님 |
| 완료 resume |raw68파일 SHA/inode/mtime/size·event2로그·선택내용·누적wall 불변, 추가capture0 |
| CPU 변환 |train2, Vehicle1/Pedestrian1GT,7D/velocity off 유지, review/failed 편입0 |
| 회귀 |375tests PASS /76.426초, py_compile/diff check PASS |

신규5tests는 원문 보존/기존좌표 호환/0seq와 exact stamp, 필드 미제공 null,
복사 독립성/JSON roundtrip, 비정상 stamp의 진단 실패 방지,
추가 속도 필드가 기존 이동/Z gate 결과를 바꾸지 않음을 확인한다.
두 단일 actor 표본의 추가 크기를 혼합/지면준비를 포함한 전체5K 비용으로 확정하지 않는다.

## 운영 영향

v123 frozen5K/6환경의 manifest·source payload는 변경하지 않았다.
v124의 bounded retry/replenish/안전 gate와 명령도 유지한다.
캡처 소스 hash는 변경됐으므로 구 output에 새 runtime을 강제 resume하지 않는다.
v124 재현은 `4d4fdb06` 기준에서 수행한다. 현재 본5K root는 미생성이며 새 runtime에서 시작할 수 있다.
v124의33셀검증은 당시 근거이고 이번 v125는2셀 smoke다. 본5K·모델 학습은 실행하지 않았다.
이 기록을 감독 학습 속도나 rosbag 평가 GT로 자동 연결하지 않는다.
추후 속도가 필요하면 원문 단위/기준점/시간 정합의 별도 계약을 먼저 정해야 한다.
