# Quantized GNSS Q/R tuning evidence

상태: **EVIDENCE**  
평가일: 2026-09-19

## 결론

대회 GNSS+IMU+GT motion localization의 production 기본값을 다음과 같이
변경한다.

```text
GNSS horizontal standard deviation R: 0.05 -> 0.10 m
Global EKF x/y process-noise diagonal Q: 0.05 -> 0.0005
predict_to_current_time: true 유지
```

이 변경은 GNSS의 인위적 random noise를 가정하는 것이 아니다. K-City 부근에서
MORAI NMEA 위경도는 동서 약 14.8 cm, 남북 약 18.5 cm 격자로 양자화된다. 기존
Global EKF는 이 cell transition을 controller-facing map pose에 과도하게 반영했다.

## 주 평가

입력은 다음 raw bag이다.

```text
rosbags/raw_sensor_w_cam/competition_full_run/
  260822_005713_competition_full_route_raw_hz_run01.bag
```

이 bag은 실제 9월 15일 현장점검 bag이 아니라 `/Competition_topic`, GPS, IMU를
함께 가진 정상 시간축의 local raw bag이다. 1x 재생에서 동일 입력을 사용했다.

| moving 구간 p95 | 기존 Q/R | 채택 Q/R | 변화 |
|---|---:|---:|---:|
| 횡 위치 오차 | 7.524 cm | 6.945 cm | -7.7% |
| 횡 frame jump | 5.432 cm | 1.444 cm | -73.4% |
| 종 위치 오차 | 22.098 cm | 20.489 cm | -7.3% |
| 종 frame jump | 7.234 cm | 2.416 cm | -66.6% |
| yaw 오차 | 0.120 deg | 0.114 deg | -5.1% |
| state rate | 49.999 Hz | 50.001 Hz | 유지 |

즉 smoothing을 위해 위치 정확도를 희생한 결과가 아니다. 이 bag에서는 종·횡 위치
오차와 frame jump가 모두 개선됐다.

Canonical production 파일을 변경한 뒤 같은 bag의 15초 구간을 1x로 다시 실행한
smoke 결과는 다음과 같다.

```text
/localization/kinematic_state: 49.986 Hz
lateral frame-jump p95:        1.418 cm
longitudinal frame-jump p95:   3.483 cm
NaN/Inf output:                없음
```

입력 bag에 포함된 일부 duplicate timestamp는 기존 adapter가 거부했으며, EKF 출력
주기와 timestamp 단조성은 유지됐다.

## 현장 근거와 한계

9월 15일 LAN-to-LAN 현장점검 bag은 현재 baseline의 실제 closed-loop 문제를
확인하는 가장 강한 근거다.

```text
rosbags/eval/localization/260915_field_inspection/
```

- fusion localization steering-rate absolute p95: 약 1.59 rad/s.
- MORAI GT localization: 약 0.34 rad/s.
- 현장 Ego 위치 이동거리와 `velocity.x` 적분거리 비율: 약 1.000.

그러나 이 bag에는 `/Competition_topic`, raw GPS, raw IMU가 없어 채택 Q/R로
localization을 다시 실행할 수 없다. 따라서 이번 값은 offline production 기본값으로
채택하되, 다음 LAN-to-LAN 주행에서 동일 controller로 closed-loop 확인해야 한다.

최근 한 PC에서 MORAI+BEVFusion+planning을 함께 실행해 MORAI real-time factor가
약 0.82가 된 bag은 종방향 motion tuning 근거로 사용하지 않았다.

## 유지한 계약

- `/Competition_topic` heading 및 `velocity.x`에 임의 scale을 적용하지 않는다.
- heading correction `-1.347 deg`를 유지한다.
- local/global EKF 모두 2D mode를 유지한다.
- UDP ROS-host receipt-time 정책을 유지한다.
- 개발 중 initial pose는 `/Ego_topic`을 사용한다. 대회 전 공식 start-pose
  interface로 교체한다.
- `predict_to_current_time=true`를 유지한다.

## 남은 별도 문제

이번 Q/R 변경은 정상 GNSS 구간의 cell-transition correction을 줄이는 조치다.
다음 문제를 해결한 것으로 간주하지 않는다.

- GPS 음영 복귀 시 큰 innovation의 gradual realignment.
- checkpoint 순간이동과 일반 GNSS reacquisition 구분.
- 한 PC에서 MORAI가 비실시간으로 느려지는 개발 환경의 시간축.
- 실제 현장 controller closed-loop 승인.

Rollback은 이 커밋 직전의 다음 두 값이다.

```text
GNSS horizontal standard deviation: 0.05 m
Global EKF x/y process-noise diagonal: 0.05
```
