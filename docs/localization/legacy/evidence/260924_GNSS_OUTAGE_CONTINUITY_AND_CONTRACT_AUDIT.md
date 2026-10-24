# GNSS 음영 연속성 및 Localization 계약 감사

상태: **EVIDENCE**

실험일: 2026-09-24  
브랜치: `localization/competition-stability-v2`  
변경 전 기준: `0eeac16c`

## 결론

최초 유효 GNSS z 이후 GPS가 끊겨도 z만 마지막 값으로 유지하고 planar fused state를
계속 발행하도록 수정했다. Production Q/R, heading correction, 초기 pose 계약과 UDP
receipt-time 정책은 변경하지 않았다.

초기 pose의 `/Ego_topic` 사용은 주최 측의 공식 시작 position interface가 아직
확정되지 않아 한 번만 사용하는 의도적인 임시 계약이다. Competition Status에는 global
position이 없으므로 이를 localization source 불일치나 fallback 문제로 취급하지 않는다.
공식 시작 position을 받으면 initializer의 입력만 교체한다.

## GPS 음영 중 state 연속성

### 원인

Global EKF는 GNSS가 없어도 motion input으로 계속 예측할 수 있었지만, 최종 publisher가
GNSS z age `0.5 s` 초과 시 Global odometry 전체를 거부했다. z는 2D localization의
부가적인 지도 높이인데도 controller-facing planar state를 막고 있었다.

### 수정 계약

- 최초 유효 z가 오기 전: state readiness를 만족하지 않으므로 대기.
- 유효 z를 한 번 받은 뒤 stale: 마지막 z와 z covariance를 유지.
- x/y/yaw/twist: Global EKF 결과를 계속 발행.
- GNSS 복귀: fresh z로 자동 복귀하고 로그를 남김.
- non-finite timestamp/값: 계속 거부.

### 실제 bag replay

`gps_blank1/2`는 2026-07-29 로컬 MORAI에서 취득한 구형 raw sensor bag이며 당시
launcher와 MR/MRP provenance는 복구되지 않았다. `/Competition_topic`이 없으므로
현재 회귀시험에서만 `gt_source=ego`, heading offset `0 deg`, sim-time replay를
사용했다. 이는 취득 당시 MRP였다는 뜻이 아니다.

| bag | zero-GNSS 구간 | state mean Hz | state max gap | 결과 |
|---|---:|---:|---:|---|
| `gps_blank1.bag` | `5.636 s` | `49.997` | `26.93 ms` | 연속 발행 |
| `gps_blank2.bag` | `11.013 s` | `50.005` | `31.04 ms` | 연속 발행 |

장기 음영에서도 NaN이나 수 초 단위 output gap은 없었다. 기존 low-Q Global EKF의
재획득 구간은 두 bag에서 폭발적으로 악화하지 않았으므로, 근거 없이 covariance
inflation이나 correction clamp를 추가하지 않았다.

최신 경량 폐루프 `260919_231906...bag`도 마지막에 GPS 음영으로 진입해 그 안에서
정지했다. zero-GNSS는 종료까지 `27.746 s`였고, 구형 final publisher는 음영 진입
약 `0.44 s` 뒤부터 state를 발행하지 않았다. 이는 이번 수정 대상이 실제 폐루프에서도
발생했음을 보여준다. 다만 GPS가 복귀하기 전 bag이 끝나므로 reacquisition 근거는 아니다.

## 전체 경로 회귀

REPLAY-A 전체 99.43초를 2배속 sim-time으로 재생했다. 초기 pose는 현재 개발 계약대로
`/Ego_topic`을 사용했고, 평가 oracle에는 MR 계약과 같은 `-1.347 deg`를 적용했다.

| 지표 | p50 | p95 |
|---|---:|---:|
| 횡 오차 | `1.82 cm` | `6.93 cm` |
| 종 오차 | `9.19 cm` | `20.52 cm` |
| yaw 오차 | `0.013 deg` | `0.120 deg` |
| 횡 frame jump | `0.252 cm` | `1.449 cm` |
| 종 frame jump | `0.491 cm` | `2.388 cm` |

- final state: `4,941`개
- mean rate: `49.709 Hz`
- max state gap: `40.24 ms`

## Frame·단위 감사

REPLAY-A raw 입력과 현재 converter를 함께 확인했다.

- Competition/Ego source frame: `morai_map`.
- IMU frame: `imu`, GPS frame: `gps`.
- `velocity.x` 대비 Ego 위치 미분 속도 중앙 비율: `1.0003`.
  - `m/s` 계약이며 km/h 재변환은 금지한다.
- heading 미분과 IMU yaw-rate 상관계수: `0.9906`.
  - degree heading을 radian yaw로 변환하는 축·부호가 일치한다.
- Competition과 Ego의 heading/vx 중앙 차이: 수치상 `0`.
- GPS x/y lever arm: 현재 local/LAN sensor config 모두 `0 m`.
- raw GNSS와 Ego map XY 오차: p50 `7.87 cm`, p95 `21.16 cm`.
  - 확인된 GNSS 양자화와 일치하며 origin 또는 m/cm 단위 오류 증거는 없다.

## 남은 gate

- GPS 음영 미션 폐루프에서 steering과 trajectory 연속성 확인.
- 공식 시작 position interface로 임시 Ego initial pose 교체.
- checkpoint teleport와 정상 GNSS 재획득을 구분하는 reset 계약.
- Global correction을 controller에 직접 전달하지 않는 local-control frame prototype.
- 실제 최신 Competition message의 yaw-rate와 IMU yaw-rate 통제 A/B.
