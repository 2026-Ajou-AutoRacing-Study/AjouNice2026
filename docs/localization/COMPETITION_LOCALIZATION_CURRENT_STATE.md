# Competition Localization 현재 계약

상태: CURRENT. 코드 대조·문서 정리: 2026-10-24. 보존된 주행 근거의 마지막 갱신은
2026-10-15이며, 이번 정리로 LAN/공식 checkpoint 검증이 추가된 것은 아니다.

## Production

| 항목 | MR·실대회 | MRP |
|---|---|---|
| alias / motion source | `localization` / `/Competition_topic` heading, velocity.x | `localization_mrp` / `/Ego_topic` |
| heading correction | `-1.347 deg` | `0 deg`; 시나리오·지도 yaw 독립 대조 미완료 |
| 초기 위치 / yaw | 개발 기본: 첫 Ego **위치만** / 보정된 Competition heading | 첫 Ego 위치·heading |
| GNSS horizontal sigma / Global Qxy | `0.10 m` / `0.0005` | 동일 |
| Global-only soft body vy prior | ON, sigma `0.20 m/s` | 동일 기본값; MRP 폐루프 미검증 |
| Smooth longitudinal / lateral / yaw tau | `0.5 / 0.1 / 0.5 s`, vehicle pivot | 동일 |
| lateral correction rate cap | `0.3 m/s`, 차량 운동이 아닌 **Global 위치 보정 성분**에만 적용 | 동일 |
| GNSS motion compensation | `gnss_motion_compensation_sec=0.0` (OFF) | 동일 |
| relocation guard | `enable_relocation_guard=false`, monitor-only 기본 true | 동일 |
| quantization-aware control candidate | `use_quantized_gnss_control_state=false` | 동일 |

공통 고정 계약:

- status heading/vx와 IMU yaw-rate/ax는 Local/Global EKF에, GPS map XY는 Global에만 전달한다.
  IMU ay mask는 이미 OFF다. soft vy prior는 실측 Competition vel_y가 아닌 모델 prior다.
  Local에는 이 prior를 넣지 않으며, launch 인자가 publisher와 Global mask를 함께 확정한다.
- `two_d_mode=true`, `predict_to_current_time=true`, `publish_sensor_tf=false`.
- [UDP receipt-time](morai_udp_receipt_time.md), `/use_sim_time=false`가 live 계약이다.
  receipt-time은 userspace dequeue 시각이지 물리 measurement/NIC 시각이 아니다.
- provider는 한 번에 하나만 실행한다. 대회에서 Ego 직접 GT provider는 fallback이 아니다.
- heading correction은 MR convention 보정이다. 이미 보정된 localization 출력에 재적용하지 않는다.
  raw Ego oracle을 평가할 때만 해당 실행의 estimator와 같은 offset을 준다.
- production `velocity.x` scale, Q/R, UDP timestamp를 임의 변경하지 않는다.

## 출력·TF·GPS 음영

Local EKF는 `odom -> base_link`를 발행한다. 최종 publisher는 Local pose를 Global
stamp로 외삽해 `map -> odom`을 합성하며, EKF stamp를 보존한 map-frame base_link
Odometry `/localization/kinematic_state`를 발행한다. 최신 TF를 무시각 조회해 합성하지 않는다.

vehicle-pivot Smooth는 같은 시각 Local 운동과 Global correction을 결합해
`/localization/control_kinematic_state`를 발행하며 TF는 발행하지 않는다.
Planning/TF의 Global 출력 계약은 유지한다. Local odometry만 controller로 단순 remap하면
map trajectory와 frame이 달라지므로 금지한다. 이 문서는 planning/control 정책을 변경하지 않는다.

GPS `(0,0)`은 거부한다. 최초 유효 z 이후 GPS가 stale/invalid이면 마지막 z만 유지하고
Global의 planar state를 계속 발행한다. 최초 z조차 없으면 startup-ready가 아니다.
state 약50Hz 연속성과 실제 위치 정확도는 별개다.

## 초기화·checkpoint

공식 START 위치 후보는 명시적으로 선택한다.

```bash
localization initial_position_source:=official_start
```

v85 규칙의 `start_finish.xyz`를 `morai_map -> map`으로 변환하고 yaw는 보정된
Competition heading을 사용한다. 공식 yaw와 차이 `1 deg` 초과, START와 GNSS 거리
`3 m` 초과, GNSS/heading stamp 차이 `0.5 s` 초과면 초기화를 거부한다.
이 모드는 Ego를 구독/대기하지 않는다. MR 로컬 출발 정합은 통과했으나 LAN 검증 전까지
기본값이 아니다. 사고 후 복귀를 START 초기화로 반복 처리하지 않는다.

보존된 운영 기록은 수정안 START+CP1–14와 Spawn 툴0–14가 일치한다고 정정했다.
원본 실행명 CP13/15는 구번호(현행12/14)이므로 원문을 고쳐 쓰지 않는다.
변경 전 로컬 툴14에서는 raw NMEA zero가 관측됐다. 운영측은 CP14 좌표·spawn을
유지하고 음영 범위를 조정하겠다고 답했으나 **새 환경 배포·GPS 수신·재출발 검증은 대기**다.
[공식 안내 보관](../competition/261015_공식_체크포인트_안내_보관.md)과 로컬 관측을 구분한다.
runtime rule monitor의 구15점 표 동기화는 이 정리 범위에서 수행하지 않았다.

유효 GNSS 복귀에서 Global/Smooth 정합 대기, fresh route, 기존 제어 재개를 검증하는
것이 우선이다. GPS도 새 절대위치도 없는 teleport는 soft prior/ZUPT/Q 조정으로 복구할 수 없다.
개발 helper의 known-checkpoint prior는 명시적 실험 입력이며 대회 기본 입력이 아니다.

guard는 OFF/monitor-only를 유지한다. active 실험은 정차 1초·GNSS 위치 안정 1초,
fresh/frame/quaternion/covariance 검사, 5m innovation 3표본, Global ACK를 요구한다.
timeout/발행 실패는 blocked latch다. Local XY를 강제 reset하지 않으며 heuristic은
공식 checkpoint 이벤트나 자동 제어 재개 허가가 아니다.

## 근거와 남은 gate

[CURRENT_EVIDENCE](CURRENT_EVIDENCE.md)는 현재 채택 근거와 역사적 후보를 구분한다.
MR 실제 soft/Smooth 완주·음영 pair·30초 Park hold 근거는 있지만 LAN, MRP 별도 폐루프,
slip/충돌, 변경 환경 CP14 및 배우·미션상태를 포함한 checkpoint 복귀는 미완료다.
20ms GNSS 보상은 입력별 개선/악화가 역전돼 기본 0초를 유지한다.

변경 승인 순서: 동일 raw bag offline A/B → `od_gt` 경량 폐루프 → GPS 음영
진입·유지·복귀 → LAN-to-LAN 동일 controller → production 승격.
횡/종 오차·frame jump·yaw·rate·steering-rate/sign flip을 함께 본다.
2026-10-15 soft/개선 Smooth는 사용자 승인으로 기본값이 됐으며, 승인과 로컬 결과를
미완료 LAN/slip gate의 통과로 기록하지 않는다.

Fast-LIO/직접 GT/구형 GNSS provider와 opt-in 연구 node는 설치·공통 launch·테스트
호환성 때문에 남아 있다. 그 경로가 존재한다는 사실은 production 선택 근거가 아니다.
