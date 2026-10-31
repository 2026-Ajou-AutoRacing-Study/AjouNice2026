# Localization motion input ablation

상태: **EVIDENCE / REJECTED CHANGE**
검증일: 2026-09-25

## 결론

Competition 절대 heading·vx와 함께 쓰는 IMU yaw-rate·ax가 상관 입력이라 EKF를
악화하는지 실제 filter replay로 검증했다. yaw-rate 제거는 yaw 성능을 명확히
악화했고, ax 제거는 장주행에서 의미 있는 이득 없이 종오차를 조금 악화했다.
따라서 production의 `heading + vx + IMU yaw-rate + IMU ax` 구성을 유지한다.

## 평가 계약

- 입력: 기록된 `/Competition_topic`, `/sensor/gps`, `/sensor/imu`.
- oracle: `/Ego_topic`, MR heading offset `-1.347 deg`.
- 현재 production sigma `0.10 m`, Global Qxy `0.0005`.
- filter를 1× sim-time으로 새로 실행하고 output만 비교했다.
- controller 명령은 다시 계산하지 않았으므로 폐루프 성능 근거가 아니다.

4× 사전 실행에서는 입력 topic은 기록됐지만 final state가 약 `24.8 Hz`로 떨어졌다.
이는 production 50 Hz 계약을 재현하지 못하므로 전부 폐기했다. Localization filter
비교는 output state rate가 약 50 Hz인지 확인한 1× replay만 유효하다.

## 52초 production bag 2×2 ablation

단위는 위치·jump `cm`, yaw `deg`이며 각 칸은 `p50 / p95`다.

| 입력 | 횡 오차 | 종 오차 | 횡 jump | 종 jump | yaw 오차 | state rate |
|---|---:|---:|---:|---:|---:|---:|
| baseline: yaw-rate+ax | 1.526 / 5.472 | **1.420 / 5.536** | 0.182 / 1.297 | 1.601 / 7.281 | **0.005 / 0.094** | 49.999 Hz |
| no ax | **1.516 / 5.425** | 1.535 / 5.724 | 0.181 / 1.286 | **1.583 / 7.118** | **0.005 / 0.088** | 49.980 Hz |
| no yaw-rate | 1.508 / 5.451 | 1.464 / 5.700 | **0.177 / 1.281** | 1.614 / 7.133 | 0.023 / 0.342 | 49.917 Hz |
| heading+vx only | 1.522 / 5.455 | 1.523 / 5.742 | 0.183 / 1.283 | 1.619 / 7.151 | 0.022 / 0.367 | 49.957 Hz |

yaw-rate를 빼면 yaw p95가 약 3.6~3.9배 악화했다. ax 제거는 작은 mixed trade-off라
장주행으로 한 번 더 확인했다.

## 242초 production 장주행 확인

| 입력 | 횡 오차 | 종 오차 | 횡 jump | 종 jump | yaw 오차 | state rate |
|---|---:|---:|---:|---:|---:|---:|
| baseline | 1.816 / 5.975 | **1.886 / 6.794** | 0.210 / 1.311 | 2.021 / 8.578 | 0.006 / **0.057** | 49.974 Hz |
| no ax | 1.821 / 5.952 | 1.965 / 6.841 | 0.210 / 1.309 | 2.064 / 8.406 | 0.006 / 0.059 | 49.975 Hz |

ax 제거의 횡 p95 차이는 `0.023 cm`, 횡 jump p95 차이는 `0.002 cm`로 실질적
이득이 아니다. 반면 종 p50/p95는 모두 악화했다. production 변경을 기각한다.

## 후속 실험

새 bag에 `/competition/raw_twist`가 들어오면 Competition yaw-rate와 IMU yaw-rate를
서로 대체하는 A/B를 수행한다. 같은 simulator state에서 파생됐을 수 있는 둘을 독립
측정처럼 동시에 fuse하지 않는다.

재현 도구:

```bash
scripts/evaluation/localization/run_motion_input_ablation.sh INPUT.bag
```

기본 네 profile을 1×로 실행한다. 일부 profile만 재검증하려면
`LOCALIZATION_ABLATION_PROFILES`에 공백으로 구분한 `name:yaw_rate:ax`를 전달한다.
