# Localization fusion input contract audit

상태: **EVIDENCE / KEEP CURRENT INPUTS**  
검증일: 2026-09-25

## 결론

현재 production의 `Competition heading/vx + IMU yaw-rate/ax` 입력 계약을 유지한다.

- 최신 receipt-time bag에서 IMU yaw-rate는 Competition heading 미분과 거의 동일한
  부호·단위·시간축을 보였다.
- IMU 장착 위치 보정은 `robot_localization`이 TF를 이용해 이미 수행한다. 따라서
  `imu_covariance_adapter`에 별도 centripetal/tangential correction을 추가하면
  이중 보정이 된다.
- 저장된 bag에는 `/competition/raw_twist`가 없어 Competition packet yaw-rate와의
  직접 대체 A/B는 아직 할 수 없다.
- 이 감사로 production launch, Q/R, fusion input은 변경하지 않았다.

## 평가 provenance

| 항목 | 최신 경량 폐루프 bag | 구 raw full-route bag |
|---|---|---|
| bag | `260919_231906_q0005_r010_gt_object_lightweight_no_traffic.bag` | `260822_005713_competition_full_route_raw_hz_run01.bag` |
| 실행 환경 | MR, production Q/R 이후 | MR, receipt-time 정리 전 과거 수집 |
| heading source | `/Competition_topic` | `/Competition_topic` |
| yaw-rate source | `/sensor/imu` | `/sensor/imu` |
| heading correction | 미분에는 상수 offset 영향 없음 | 동일 |
| 비교 방법 | unwrap heading, 7-sample local regression, lag `+-0.1 s` sweep | 동일 |

재현 명령:

```bash
python3 scripts/evaluation/localization/analyze_competition_yaw_rate.py INPUT.bag
```

평가기에서 `/competition/raw_twist`는 선택 입력이다. 해당 토픽이 없는 기존 bag도
IMU 대 heading 미분 진단을 수행하며, Competition yaw-rate 비교만 blocked로 남긴다.

## yaw-rate 정합 결과

| bag | best lag | correlation | MAE | p50 | p95 | RMSE |
|---|---:|---:|---:|---:|---:|---:|
| 최신 receipt-time | `+22 ms` | **0.99925** | `0.00408` | `0.00217` | **0.01402** | `0.00750` |
| 구 raw full-route | `+32 ms` | 0.99758 | `0.01418` | `0.00986` | `0.03810` | `0.02486` |

오차 단위는 `rad/s`다. 최신 bag의 p95는 구 bag 대비 약 `63%` 작다. 이는 현재
IMU yaw-rate의 frame/sign/unit이 올바르며, receipt-time 이후 timestamp 품질도
개선됐다는 근거다. best lag는 heading을 7개 표본으로 미분하는 과정과 topic 수신
위상까지 포함하므로 별도 hard-coded time offset으로 보정하지 않는다.

## timestamp 품질

| bag/topic | message | non-increasing stamp | unique effective rate |
|---|---:|---:|---:|
| 최신 Competition | 12,083 | **0** | 49.811 Hz |
| 최신 IMU | 12,024 | **0** | 49.582 Hz |
| 최신 GPS | 7,250 | **0** | 29.896 Hz |
| 구 Competition | 4,248 | 1,280 | 29.838 Hz |
| 구 IMU | 4,201 | 1,369 | 28.483 Hz |
| 구 GPS | 2,636 | 0 | 26.511 Hz |

구 bag은 Competition/IMU header가 중복되거나 역행한 표본이 많다. 현재 adapter의
duplicate/out-of-order rejection과 UDP receipt-time 계약을 유지해야 한다.

## IMU lever-arm 판단

현재 MORAI IMU TF는 차량 원점에서 전방 약 `1.528 m`에 있다. 회전 중 센서 위치에는
구심·접선 가속도가 생기므로 처음에는 adapter 보정 후보를 검토했다. 그러나
`robot_localization`의 IMU acceleration 준비 단계가 TF translation과 angular
velocity/acceleration을 사용해 센서 위치 가속도를 target frame 원점으로 이미
변환한다.

따라서 다음 변경은 기각한다.

```text
imu_covariance_adapter에서 별도 lever-arm acceleration correction 추가
```

TF가 올바르게 발행되는 것이 계약이며, 같은 보정을 adapter에 복제하지 않는다.

## 남은 gate

새 MR 또는 LAN-to-LAN bag에는 다음 토픽을 포함한다.

```text
/Competition_topic
/competition/raw_twist
/sensor/imu
/sensor/gps
/localization/kinematic_state
```

그 뒤 Competition yaw-rate와 IMU yaw-rate를 각각 **대체 입력**으로 A/B한다. 둘을
독립 센서처럼 동시에 fuse하지 않는다. 새 bag 전까지 IMU yaw-rate와 ax를 유지한다.
