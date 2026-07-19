# Control 속도 단위 규약: km/h와 m/s

전체 MORAI 제어 실행 순서는 [MORAI Control Pipeline 실행 방법](how_to_launch_control_pipeline.md)을
참고한다.

## 1. 결론

제어 파이프라인의 **내부 표준 단위는 m/s**다. 사람이 설정하는 주행 속도와 지도 제한속도만
km/h를 사용하고, trajectory를 생성할 때 m/s로 변환한다.

현재 MORAI GT localization은 예외적으로 `/localization/kinematic_state`의 속도 숫자를
km/h 그대로 전달한다. 따라서 Follower와 Validator가 이 입력을 사용하는 경계에서 각각
`1 / 3.6`을 곱해 m/s로 정규화한다.

```text
trajectory_generator YAML (km/h)
        │  min(희망 속도, 제한속도) / 3.6
        ▼
/control/trajectory.longitudinal_velocity_mps (m/s)
        │
        ├───────────────┐
        ▼               ▼
Follower             Validator
        ▲               ▲
        │ × 1/3.6       │ × 1/3.6
/localization/kinematic_state (MORAI GT 사용 시 숫자는 km/h)
```

## 2. 문제가 발생했던 이유

MORAI 차량이 실제로 약 17 km/h로 주행할 때 kinematic state의 속도 숫자도 약 `17`이었다.
이 값을 m/s라고 가정하면 다음과 같이 해석된다.

```text
잘못된 해석: 17 m/s = 61.2 km/h
정상 변환:   17 km/h / 3.6 = 약 4.72 m/s
```

목표 속도는 trajectory에서 이미 m/s인데 실제 속도만 km/h 숫자를 그대로 사용하면서,
속도 오차가 실제보다 약 3.6배 크게 계산됐다. 그 결과 PI 종방향 제어의 가감속 판단과
Validator의 속도 오차·응답 분석이 잘못됐다.

이 문제는 좌표계나 Follower 튜닝 문제가 아니라 **입력 경계의 단위 불일치**였다. 단위가
틀린 상태에서 PI gain을 조정하면 잘못된 입력에 맞춰 튜닝하게 되므로 단위부터 확인해야 한다.

## 3. 구간별 단위

| 구간 또는 필드 | 단위 | 비고 |
| --- | --- | --- |
| `desired_velocity_kph` | km/h | 운전자가 요청하는 순항 속도 |
| `speed_limit_kph` | km/h | lanelet/drive-course의 최대 허용 속도 |
| `default_speed_limit_kph` | km/h | 지도에 제한속도가 없을 때의 안전 상한 |
| trajectory CSV `velocity_mps` | m/s | Generator가 km/h에서 변환해 저장 |
| `/control/trajectory.*.longitudinal_velocity_mps` | m/s | Follower와 Validator의 목표 속도 |
| MORAI GT `/localization/kinematic_state.twist.twist.linear.*` | km/h 숫자 | 현재 provider의 예외적인 계약 |
| Follower 내부 `current_velocity`, `target_velocity` | m/s | kinematic 입력 변환 후 계산 |
| Validator의 `*_mps` CSV 열과 metric | m/s | kinematic 입력 변환 후 기록 |
| `/ctrl_cmd.velocity` | 현재 type-1 제어에서는 사용하지 않음 | accel/brake/steering을 사용 |

Trajectory Generator의 실제 목표 속도는 다음과 같다.

```text
final_velocity_mps = min(desired_velocity_kph, speed_limit_kph) / 3.6
```

`speed_limit_kph`는 목표 속도가 아니라 상한이다. 예를 들어 희망 속도 20 km/h, 제한속도
60 km/h이면 trajectory 목표는 20 km/h, 즉 약 5.56 m/s다.

## 4. 현재 변환 설정

Follower 설정:

```yaml
# base_ws/src/control/trajectory_follower/config/trajectory_follower.yaml
kinematic_velocity_scale_to_mps: 0.2777777777777778
```

Validator 설정:

```yaml
# base_ws/src/control/trajectory_validator/config/trajectory_validator.yaml
input:
  kinematic_velocity_scale_to_mps: 0.2777777777777778
```

두 값은 같은 입력을 각 프로세스 내부에서 독립적으로 변환한다. Follower가 변환한 메시지를
Validator에 재전송하는 구조가 아니므로 중복 변환이 아니다.

## 5. Localization provider를 바꿀 때

Fast-LIO 등 새 provider가 `/localization/kinematic_state`를 정상적인 m/s로 발행한다면 두
scale을 모두 `1.0`으로 바꿔야 한다.

```yaml
kinematic_velocity_scale_to_mps: 1.0
```

Validator는 다음과 같이 설정한다.

```yaml
input:
  kinematic_velocity_scale_to_mps: 1.0
```

m/s 입력에 `1/3.6`을 또 적용하면 실제 속도가 3.6배 작게 계산된다. 반대로 km/h 입력에
`1.0`을 사용하면 3.6배 크게 계산된다. Provider를 변경할 때는 Follower와 Validator 설정을
반드시 함께 변경한다.

## 6. 실행 전 점검 방법

### 목표 속도 확인

```bash
rostopic echo -n 1 /control/trajectory/points/0/longitudinal_velocity_mps
```

20 km/h로 생성한 경로라면 약 `5.56` m/s가 나와야 한다. 토픽 구조상 위 field 경로가
지원되지 않으면 전체 메시지에서 첫 point를 확인한다.

```bash
rostopic echo -n 1 /control/trajectory
```

### MORAI GT 입력 확인

```bash
rostopic echo -n 1 /Ego_topic
rostopic echo -n 1 /localization/kinematic_state/twist/twist/linear
```

MORAI 계기판이 약 18 km/h일 때 kinematic 값이 약 `18`이면 해당 provider는 km/h 숫자를
전달하는 것이므로 scale은 `1/3.6`이어야 한다. 값이 약 `5.0`이면 이미 m/s이므로 scale은
`1.0`이어야 한다.

### 실행 중 정규화 결과 확인

Validator를 기록 중이라면 rqt에서 다음 값을 비교한다.

```text
/trajectory_validator/debug/ego_velocity_mps
/trajectory_validator/debug/target_velocity_mps
/trajectory_validator/debug/velocity_error_mps
```

17 km/h 주행이라면 `ego_velocity_mps`는 약 `4.72`가 나와야 한다. 저장된
`control_cycles.csv`, `vehicle_responses.csv`의 이름이 `*_mps`인 열도 모두 m/s다.

## 7. 이상 증상별 확인 사항

| 증상 | 가장 먼저 확인할 항목 |
| --- | --- |
| 실제 17 km/h인데 분석 값이 약 17 m/s | scale이 `1.0`으로 설정됐는지 확인 |
| 실제 17 km/h인데 분석 값이 약 1.31 m/s | m/s 입력에 `1/3.6`을 중복 적용했는지 확인 |
| 목표 속도가 제한속도와 항상 같음 | `desired_velocity_kph`와 `speed_limit_kph` 역할 확인 |
| 속도 오차만 비정상적으로 큼 | 목표와 실제 속도가 같은 단위인지 먼저 확인 |
| Follower와 Validator 속도가 서로 다름 | 두 패키지의 scale 값이 동일한지 확인 |

## 8. 변경 시 원칙

- 메시지 또는 CSV에서 이름이 `*_mps`이면 반드시 m/s 값을 저장한다.
- km/h 설정은 이름에 `_kph`를 붙인다.
- 단위 변환은 입력 경계에서 한 번만 수행한다.
- Localization provider를 변경할 때 단위를 추측하지 말고 실제 주행 값으로 검증한다.
- 단위 검증 전에는 PI gain이나 속도 오차 임계값을 튜닝하지 않는다.
