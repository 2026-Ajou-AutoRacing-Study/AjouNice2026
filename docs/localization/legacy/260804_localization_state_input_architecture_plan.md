# Localization state·input 구조 개선 검토 계획

## 1. 문서 목적

현재 GNSS+IMU+GT 측위는 ROS Noetic의 `robot_localization`을 사용한다.
이 방식은 ROS 메시지, TF, 지연 측정, 센서 queue와 공분산 처리가
안정적이라는 장점이 있지만, 필터 상태와 센서 역할이 대회 차량 모델에
비해 일반적이고 크다.

이 문서는 다음 변경을 바로 구현하기 위한 명세가 아니라, 기존 구현을
기준선으로 유지하면서 향후 상태와 입력 구조를 어떻게 개선할지 결정하기
위한 계획이다.

- `robot_localization`의 고정 15-state를 계속 사용할 것인가
- 평면 차량에 필요한 작은 상태만 사용하는 EKF를 만들 것인가
- IMU를 Kalman 측정으로 사용할 것인가, 운동 모델 입력으로 사용할 것인가
- 무잡음 GT heading과 종방향 속도를 얼마나 강하게 반영할 것인가
- 기존 패키지를 fork할 것인가, 별도 localization 패키지를 만들 것인가

## 2. 현재 구현

현재 필터의 전체 상태는 다음 15개다.

\[
\mathbf{x} =
[x,y,z,roll,pitch,yaw,v_x,v_y,v_z,
\omega_x,\omega_y,\omega_z,a_x,a_y,a_z]^T
\]

`*_config`의 `true/false`는 상태를 추가하거나 삭제하지 않는다.

```text
true  : 해당 센서 메시지의 성분을 측정 업데이트에 사용
false : 해당 센서 메시지의 성분을 측정 업데이트에서 제외
```

따라서 `false`인 상태도 내부 상태 벡터와 15×15 상태 공분산 `P`에는
계속 존재한다. 측정하지 않는 상태도 예측 모델에서 생긴 교차 공분산을
통해 다른 측정 업데이트의 영향을 간접적으로 받을 수 있다.

현재 GT 보조 global EKF가 직접 측정으로 사용하는 성분은 다음 6개다.

| 입력 | 사용 성분 | 역할 |
|---|---|---|
| GNSS odometry | `x`, `y` | map 기준 절대 위치 보정 |
| GT orientation | `yaw` | 절대 heading 보정 |
| GT twist | `vx` | 차량 종방향 속도 보정 |
| IMU | `wz`, `ax` | 회전·가감속 상태 보정 |

`two_d_mode: true`이므로 `z`, roll, pitch와 이에 대응되는 3D 속도·가속도
성분은 평면 주행에 맞게 0으로 제약된다. 그러나 `vy`, `ay`는 평면 상태에
속하므로 15-state 내부에 남는다.

현재 방식에서 IMU `wz`, `ax`는 엄밀히 말해 운동 모델의 외부 입력이
아니다. 먼저 15-state의 `wz`, `ax`를 Kalman measurement update한 뒤,
그 추정 상태가 다음 prediction에 사용된다.

## 3. 변경하려는 센서 역할

장기적으로 원하는 역할 분리는 다음과 같다.

```text
IMU ax, wz
  -> 운동 모델 propagation 입력
  -> IMU uncertainty는 process noise에 반영

GT heading, velocity.x
  -> 작은 measurement covariance를 사용한 강한 Kalman update

GNSS x, y
  -> 일반적인 위치 Kalman update
  -> 유효하지 않은 GNSS와 음영 구간은 update 차단
```

IMU를 propagation 입력으로 쓴다는 것은 상태에 IMU 값을 무조건
덮어쓴다는 뜻이 아니다. IMU로 측정 사이의 상태를 전파하고, 그
불확실성을 상태 공분산에 누적한다는 뜻이다.

GT는 대회에서 노이즈 없는 값으로 제공되므로 높은 신뢰도로 반영한다.
그러나 hard assignment는 구현하지 않는다. 아주 작은 양의 measurement
covariance를 사용하면 Kalman gain이 1에 가까워져 사실상 대입처럼
동작하면서도 다음 기능을 유지할 수 있다.

- timestamp에 맞춘 측정 처리
- innovation 및 이상값 검사
- 상태와 공분산의 일관성
- 지연·중복·일시적 비정상 값에 대한 방어
- 추후 GT가 아닌 센서로 교체할 수 있는 구조

GT covariance에는 0이 아닌 작은 하한을 둔다. 초기 검토값은 다음과
같으며, 최종값은 rosbag innovation 통계로 결정한다.

```yaml
gt_yaw_variance: 1.0e-6
gt_velocity_variance: 1.0e-4
```

## 4. 목표 운동 모델 후보

### 4.1 4-state + IMU input

가장 단순하고 현재 목적에 가까운 후보는 다음과 같다.

\[
\mathbf{x}=[x,y,\psi,v]^T
\]

\[
\mathbf{u}=[\omega_z,a_x]^T
\]

평면에서 slip이 크지 않다는 가정 아래 다음과 같이 전파한다.

\[
\psi_{k+1}=\psi_k+\omega_z\Delta t
\]

\[
v_{k+1}=v_k+a_x\Delta t
\]

\[
x_{k+1}=x_k+v_k\cos\psi_k\Delta t
\]

\[
y_{k+1}=y_k+v_k\sin\psi_k\Delta t
\]

장점:

- 상태 의미와 센서 역할이 명확하다.
- 관측하지 않는 `vy`, `ay`가 임의로 추정되는 문제를 제거한다.
- covariance와 process noise 튜닝 대상이 작다.
- GT heading과 `vx`, GNSS `x/y`의 역할을 분명하게 분리할 수 있다.

제약:

- 큰 측면 slip, 충돌 또는 의도적인 횡이동은 표현하지 못한다.
- IMU extrinsic, 중력 제거와 timestamp가 정확해야 한다.
- 기존 `robot_localization`의 ROS 처리 기능을 필요한 만큼 다시
  구현해야 한다.

### 4.2 6-state CTRA 계열

IMU 값을 입력이 아닌 추정 상태로 평활화하려면 다음 상태도 가능하다.

\[
\mathbf{x}=[x,y,\psi,v,\omega_z,a_x]^T
\]

IMU는 `wz`, `ax` 측정 update를 담당하고, 추정된 `wz`, `ax`로 상태를
전파한다.

장점:

- IMU 측정값을 필터 내부에서 평활화할 수 있다.
- 일시적인 IMU 누락에도 마지막 추정 `wz`, `ax`를 사용할 수 있다.

단점:

- 4-state보다 상태와 튜닝 항목이 늘어난다.
- IMU noise를 measurement noise와 process noise에 중복 반영하지 않도록
  주의해야 한다.
- 시뮬레이터의 중복 UDP와 낮은 고유 timestamp 갱신률을 별도로 처리해야
  한다.

현재 대회 환경에서는 4-state + IMU input을 우선 후보로 두고, 6-state는
IMU 평활화가 실제 성능상 필요할 때 비교한다.

## 5. 가능한 구현 방안

### 방안 A. 현재 `robot_localization` 유지 및 covariance 튜닝

내용:

- 기존 15-state local/global EKF를 유지한다.
- GT covariance를 작게 조정한다.
- GNSS 음영 차단, heading correction과 중복 timestamp 차단은 adapter에서
  담당한다.

장점:

- 변경량과 위험이 가장 작다.
- TF, queue, 지연 측정, diagnostics 기능을 그대로 사용한다.
- 현재 rosbag 결과와 연속적으로 비교할 수 있다.

단점:

- 상태는 계속 15개다.
- `vy`, `ay`를 실제 상태에서 제거할 수 없다.
- IMU가 원하는 의미의 외부 propagation input이 아니다.

용도:

- 현재 기준선 및 대회 fallback
- covariance 튜닝만으로 목표 성능을 만족하는지 확인

### 방안 B. `robot_localization` fork 후 핵심부 수정

내용:

- 팀 저장소로 패키지를 fork한다.
- 상태 크기, 상태 인덱스, 전이 모델과 IMU 처리 방식을 수정한다.
- 기존 ROS wrapper, TF, queue, diagnostics 기능은 최대한 재사용한다.

장점:

- 검증된 ROS 통합 기능을 재사용할 수 있다.
- 기존 launch와 topic interface를 비교적 비슷하게 유지할 수 있다.

단점:

- `robot_localization`은 여러 부분에서 고정 15-state를 전제로 한다.
- 단순히 `STATE_SIZE`만 바꾸는 것으로 끝나지 않는다.
- 상태 인덱스, message mapping, covariance 입출력, `two_d_mode`,
  `set_pose`, local/global TF와 테스트까지 넓게 수정해야 한다.
- upstream과 동기화하기 어려워지고 시스템 설치본과 custom build를
  혼동할 수 있다.

판단:

- 일부 동작만 수정할 때는 고려할 수 있다.
- 4-state 차량 모델을 만드는 목적에는 변경 범위가 지나치게 넓을
  가능성이 높다.

### 방안 C. 별도 planar vehicle EKF 패키지 구현

예상 구조:

```text
base_ws/src/localization/vehicle_ctra_localization/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── vehicle_ctra_ekf.yaml
├── include/vehicle_ctra_localization/
│   └── vehicle_ctra_ekf.hpp
├── src/
│   ├── vehicle_ctra_ekf.cpp
│   └── vehicle_ctra_ekf_node.cpp
└── test/
```

구성:

- 순수 C++ EKF 수학 코어와 ROS wrapper를 분리한다.
- 4-state 또는 6-state를 명시적으로 선택한다.
- IMU input propagation과 GT/GNSS measurement update를 구분한다.
- 기존 output topic과 frame 계약을 유지한다.

장점:

- 대회 차량 모델과 센서 역할을 가장 정확하게 표현할 수 있다.
- 상태와 covariance를 완전히 통제할 수 있다.
- 기존 `robot_localization` baseline과 A/B 실행 및 rollback이 쉽다.
- 작은 수학 코어를 ROS 없이 단위 테스트할 수 있다.

단점:

- timestamp queue, delayed measurement, TF와 diagnostics 등 필요한 기능을
  직접 구현하고 검증해야 한다.
- 초기 버전에서 구현 누락 가능성이 있다.

판단:

- 최종 구조로 가장 적합한 후보이다.

### 방안 D. 차량용 기존 localization stack 도입

Autoware 계열의 `ekf_localizer`처럼 평면 차량 pose/twist, yaw bias,
지연 보정과 innovation gate를 제공하는 구현을 참고하거나 도입하는
방법이다.

장점:

- 차량 운동에 맞춘 설계와 검증 사례를 참고할 수 있다.
- 지연 측정과 이상값 처리 설계가 잘 정리되어 있다.

단점:

- 현대 Autoware 구현은 주로 ROS2 생태계에 속한다.
- 현재 ROS1 Noetic workspace에 직접 넣으면 의존성과 포팅 부담이 크다.
- 현재 topic, TF, MORAI adapter 계약에 맞춘 추가 작업이 필요하다.

판단:

- 전체 stack을 바로 도입하기보다는 상태 모델, 지연 보정과 gate 설계의
  참고 구현으로 활용한다.

## 6. 권장 진행 방향

현 단계의 권장안은 방안 A를 기준선으로 유지하면서 방안 C를 별도
패키지로 개발하는 것이다.

```text
기존 robot_localization
  -> 안정적인 baseline 및 fallback

신규 planar vehicle EKF
  -> 4-state + IMU input
  -> 강한 GT Kalman update
  -> 일반 GNSS Kalman update
```

첫 신규 모델은 다음으로 제한한다.

```text
state       : x, y, yaw, longitudinal velocity
input       : IMU yaw rate, longitudinal acceleration
measurement : GT yaw, GT longitudinal velocity, GNSS x/y
output rate : 50 Hz
```

다음 기능은 초기 버전에 반드시 포함한다.

- timestamp가 같거나 과거인 입력 거부
- IMU 입력 timeout 처리
- GNSS 유효성 및 음영 차단
- GT heading correction on/off
- angle normalization
- 작은 양수의 GT covariance
- innovation/Mahalanobis gate
- 초기 pose 한 번 적용
- covariance가 포함된 `/localization/kinematic_state`
- 기존 TF publisher와의 중복 방지
- 각 센서 update 수와 reject 수 diagnostics

초기 버전에는 다음을 넣지 않는다.

- GT hard assignment
- `vy`, `ay` 상태
- IMU bias 상태
- 3D roll/pitch/z 추정
- 자동으로 `/Ego_topic`에 fallback하는 대회 모드

## 7. 검증 계획

동일한 raw rosbag과 동일한 초기 pose를 사용해 다음 두 버전을 비교한다.

```text
A: 기존 robot_localization 15-state
B: 신규 planar vehicle EKF
```

필수 비교 항목:

| 구분 | 지표 |
|---|---|
| 위치 | XY mean/RMSE/max/final |
| 차량 기준 위치 | longitudinal/lateral bias, MAE, RMSE |
| 자세 | yaw MAE/max |
| 속도 | longitudinal velocity bias/MAE/RMSE |
| 불필요한 횡상태 | 출력 `linear.y` 또는 명시적인 0 정책 |
| GNSS 음영 | 음영 진입·7.1초 구간·복구 직후 오차 |
| 주기 | wall-time 출력 Hz, min/max interval, jitter |
| 안정성 | NaN, covariance 양의 준정부호, innovation reject |
| 성능 | callback 처리시간, CPU 사용률 |

공정한 비교를 위해 다음 조건을 고정한다.

- 같은 센서 topic과 timestamp
- 같은 heading correction
- 같은 초기 pose
- 같은 평가 구간
- 같은 GT reference와 frame convention
- 같은 GNSS 음영 차단 정책
- warm-up 및 초기 유효하지 않은 IMU orientation 구간 처리 일치

## 8. 의사결정 기준

신규 EKF는 단순히 상태 수가 작다는 이유만으로 채택하지 않는다. 다음을
모두 만족할 때 기존 구현을 대체한다.

1. 정상 GNSS 구간에서 위치·yaw 성능이 기존보다 나쁘지 않다.
2. GNSS 음영에서 오차 증가가 기존보다 작거나 동등하다.
3. 잘못된 `vy`, `ay` 같은 비관측 상태가 발생하지 않는다.
4. 출력 50 Hz의 간격과 timestamp가 안정적이다.
5. TF와 `/localization/kinematic_state` 계약을 유지한다.
6. 입력 누락, 중복 UDP, 시간 역행에 안전하게 대응한다.
7. 같은 rosbag을 반복 실행했을 때 재현성이 있다.

행렬 크기 감소에 따른 연산량 감소는 부수적인 장점으로만 평가한다.
15×15 연산도 현재 C++ ROS 파이프라인에서 충분히 작기 때문에, 최종
선택은 처리속도보다 모델의 적합성·관측 가능성·유지보수성을 기준으로
한다.

## 9. 현재 결정 사항과 보류 사항

결정:

- GT hard assignment는 구현하지 않는다.
- GT heading과 `velocity.x`는 작은 covariance의 강한 Kalman update로
  사용한다.
- 신규 필터에서 IMU `wz`, `ax`는 propagation input으로 사용한다.
- 기존 `robot_localization` 구현은 비교 기준과 fallback으로 보존한다.
- 신규 구현의 첫 후보는 `x, y, yaw, v` 4-state다.

보류:

- 4-state와 6-state 중 최종 선택
- GT covariance 최종 수치
- IMU process noise를 MORAI 설정에서 자동 변환할지 여부
- local/global 두 필터를 유지할지 global 단일 필터로 단순화할지 여부
- Autoware 지연 측정 처리 방식을 어느 수준까지 도입할지 여부
- 신규 EKF가 기존 구현을 대회 기본값으로 대체하는 시점

