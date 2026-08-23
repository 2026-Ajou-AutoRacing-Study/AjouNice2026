# GNSS + IMU + GT CTRA-like 측위 설계·구현·검증

## 1. 목적

대회에서 제공되는 차량 GT 일부 정보와 기존 센서를 결합해 다음 공통
출력을 안정적인 주기로 제공한다.

```text
/localization/kinematic_state
  type: nav_msgs/Odometry
  pose frame: map
  twist frame: base_link
```

대회 GT에는 차량 heading과 종방향 `velocity.x`가 제공된다. GNSS는 절대
위치를 제공하고, IMU yaw rate와 종방향 가속도는 GT 메시지 사이의 짧은
구간을 예측한다. 기존 `robot_localization` 패키지는 수정하지 않는다.

현재 저장된 rosbag에는 `/Competition_topic`이 없으므로 개발 단계에서는
동일한 `morai_msgs/EgoVehicleStatus` 타입인 `/Ego_topic`을 GT 입력
대체재로 사용한다. 실제 대회에서는 launch 인자만 바꾼다.

## 2. 기존 GNSS+IMU와의 차이

기존 GNSS+IMU baseline은 다음 측정을 융합한다.

```text
IMU orientation yaw
IMU angular_velocity.z
IMU linear_acceleration.x
IMU linear_acceleration.y
GNSS x, y
```

속도 직접 측정이 없으므로 IMU 가속도 적분 의존도가 상대적으로 크다.

GT CTRA-like 버전은 다음 측정을 사용한다.

```text
GT heading              -> 절대 yaw
GT velocity.x           -> 차량 종방향 vx
IMU angular_velocity.z  -> yaw 갱신 사이의 회전 예측
IMU linear_acceleration.x -> vx 갱신 사이의 가감속 예측
GNSS x, y               -> map 위치 절대 보정
```

다음 성분은 의도적으로 융합하지 않는다.

```text
IMU orientation yaw
IMU linear_acceleration.y/z
IMU angular_velocity.x/y
GT 또는 IMU의 vy/vz
roll/pitch
```

차량의 정상 주행에서는 body-frame 횡속도를 `vy ≈ 0`으로 가정한다.
map-frame 횡방향 이동은 종방향 속도와 heading으로 계산되며 GNSS y가
절대 위치를 보정한다. 큰 타이어 슬립이나 충돌에 의한 횡이동은 현재
모델의 명시적 대상이 아니다.

## 3. CTRA-like 상태 전파

이 구현은 별도의 CTRA 클래스를 작성하지 않는다. `robot_localization`의
15-state EKF에서 활성화한 측정값으로 다음과 같은 운용상 CTRA-like
전파를 구성한다.

\[
\psi_{k+1}=\psi_k+\omega_z\Delta t
\]

\[
v_{k+1}=v_k+a_x\Delta t
\]

\[
x_{k+1}=x_k+v\cos\psi\Delta t
\]

\[
y_{k+1}=y_k+v\sin\psi\Delta t
\]

GT yaw와 vx가 들어오면 예측값을 절대 측정으로 다시 보정하고, global
EKF에서는 GNSS x/y가 위치를 보정한다.

공통 실행 설정은 다음과 같다.

```yaml
frequency: 50
predict_to_current_time: true
two_d_mode: true
sensor_timeout: 0.2
```

`predict_to_current_time: true`는 단순 메시지 복제가 아니다. 각 50 Hz
타이머 시각까지 같은 EKF 운동 모델을 전파한다. 새 측정이 들어오면
해당 시각의 측정 업데이트가 다시 수행된다.

## 4. 토픽 흐름

```text
                    gt_source:=ego
/Ego_topic ----------------------------------+
                                                |
                    gt_source:=competition      v
/Competition_topic ------------------> morai_gt_motion_adapter.py
                                           |              |
                                           v              v
                         /localization/gt/orientation  /localization/gt/twist
                              absolute yaw              longitudinal vx
                                           |              |
/sensor/imu -> imu_covariance_adapter.py    |              |
                  |                         |              |
                  +-> /localization/imu ----+--------------+
                       wz, ax                              |
                                                           v
                                               local/global EKF

/sensor/gps -> morai_gps_to_local_odometry.py              |
                  |                                        |
                  +-> /localization/gnss/odom -------------+
                       global x/y only
```

출력:

```text
local EKF  -> /localization/odometry/local
global EKF -> /localization/odom
           -> /localization/kinematic_state
           -> /localization/acceleration
```

`/localization/acceleration`은 Global EKF가 추정한 가속도를
`geometry_msgs/AccelWithCovarianceStamped`로 발행한다.
`nav_msgs/Odometry`인 `/localization/kinematic_state`에는 가속도 필드가
없으므로 별도 토픽을 사용한다. 두 rosbag 평가 녹화 스크립트는 이 토픽도
기본 기록한다.

## 5. GT 입력 전환 정책

두 입력은 동일한 `morai_msgs/EgoVehicleStatus` 메시지 타입을 사용한다.

```text
gt_source=ego
  -> /Ego_topic

gt_source=competition
  -> /Competition_topic
```

자동 fallback은 사용하지 않는다. 실제 대회에서 Competition 입력이
끊겼을 때 자동으로 전체 Ego GT를 사용하면 규정 위반을 발견하지 못할
수 있기 때문이다. 지원하지 않는 source 값은 노드 초기화 오류로
처리한다.

GT adapter가 사용하는 필드는 다음 세 개다.

```text
header.stamp
heading
velocity.x
```

MORAI UDP receiver는 속도를 ROS SI 단위인 m/s로 변환해 발행하므로
adapter에서 3.6으로 다시 나누지 않는다. heading은 degree에서 radian과
quaternion으로 변환한다.

## 6. GT adapter 출력 형식

절대 yaw와 차량 twist를 하나의 `nav_msgs/Odometry`로 합치지 않고
분리한다.

```text
/localization/gt/orientation
  sensor_msgs/Imu
  frame_id: base_link
  사용 성분: orientation yaw

/localization/gt/twist
  geometry_msgs/TwistWithCovarianceStamped
  frame_id: base_link
  사용 성분: twist.linear.x
```

이렇게 하면 pose의 world frame과 twist의 body frame을 명확히 유지하고,
local EKF의 `odom`과 global EKF의 `map` 관계를 GT adapter가 침범하지
않는다.

adapter는 다음 데이터를 거부한다.

- zero timestamp
- 이전과 같거나 과거로 돌아간 timestamp
- NaN/Inf heading 또는 velocity
- 정규화할 수 없는 orientation

## 7. local/global EKF 역할

### Local EKF

입력:

```text
GT yaw
GT vx
IMU yaw rate
IMU ax
```

역할:

```text
상대적으로 연속적인 odom 상태
TF odom -> base_link 소유
```

### Global EKF

입력:

```text
local과 동일한 GT/IMU 운동 측정
GNSS x, y
```

역할:

```text
map 기준 절대 위치 보정
TF map -> odom 소유
```

GT adapter와 kinematic-state publisher는 TF를 발행하지 않는다.

```text
map -> odom -> base_link
```

다른 localization provider를 동시에 실행하면 같은 TF edge가 충돌하므로
FAST-LIO, 기존 GNSS+IMU, GT CTRA launch를 함께 실행하지 않는다.

## 8. 초기 pose 정책

초기 pose와 지속적인 GT 운동 입력은 별도 기능이다.

개발 단계:

```text
첫 /Ego_topic pose 한 번
  -> /localization/initialpose
  -> global EKF set_pose
```

`gt_source:=competition`으로 실행해도 초기 pose는 당분간 첫
`/Ego_topic`을 사용한다. 대회 적용 전에는
`configs/localization/localization_manager.yaml`의 초기화 source를
`manual`로 바꾸고 공식 시작 좌표를 입력한다.

초기화 이후 `/Ego_topic.position`은 EKF의 지속적인 위치 측정으로
사용하지 않는다.

## 9. covariance

설정 파일:

```text
configs/localization/gnss_imu_gt_noise_sensor6_ideal.yaml
```

noise-off MORAI 첫 검증값:

| 측정 | 표준편차 |
|---|---:|
| GT heading | 0.001745 rad (0.1 deg) |
| GT longitudinal velocity | 0.01 m/s |
| IMU yaw rate | 0.001 rad/s |
| IMU acceleration | 0.05 m/s² |
| GNSS horizontal position | 0.05 m |

0 covariance는 완전한 확신 또는 미정의로 잘못 해석될 수 있어 작지만
0이 아닌 값을 사용한다. 실제 대회 noise profile이 정해지면 반복 주행
residual을 이용해 다시 추정해야 한다.

GT longitudinal velocity는 동일한 전체 주행 raw bag에서
`0.05 m/s`와 `0.01 m/s`를 A/B 비교했다. `0.01 m/s`에서 `linear.x`
MAE가 약 31.6%, speed MAE가 약 21.6%, 종방향 위치 MAE가 약 6.5%
감소했고 전체 XY와 횡방향 오차는 사실상 유지되어 기본값으로 채택했다.

GT velocity를 `0.01 m/s`로 고정한 IMU `ax` ON/OFF 비교에서는 ON의
`linear.x` MAE가 약 30.0%, speed MAE가 약 24.0% 작았다. 가감속,
정속 및 GNSS 음영 구간에서도 같은 방향이 확인되어 IMU `ax` 융합을
유지한다. 다만 rare velocity step은 ON에서 더 커질 수 있으므로
`ax` 표준편차 `0.01 / 0.05 / 0.10 m/s²`의 후속 비교 여지는 남긴다.
세부 결과는
`rosbags/eval/localization/gnss_imu_gt_ctra_ax_ab_v1/README.md`에 있다.

ax를 유지한 상태에서 표준편차 `0.01 / 0.05 / 0.10 m/s²`를 추가
비교했다. `0.05 m/s²`는 전체 `vx`와 speed MAE가 가장 작았고,
`0.01 m/s²`보다 속도 RMSE와 rare velocity step을 줄이면서
`0.10 m/s²`보다 acceleration 측정 추종성을 더 유지했다. 따라서
균형값인 `0.05 m/s²`를 기본값으로 채택했다. 세부 sweep 결과는
`rosbags/eval/localization/gnss_imu_gt_ctra_ax_sigma_sweep_v1/README.md`에
있으며, 최종 v3 결과는
`rosbags/eval/localization/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3/README.md`에
있다.

## 10. 관련 파일

새 파일:

```text
base_ws/src/localization/localization_manager/scripts/morai_gt_motion_adapter.py
base_ws/src/localization/localization_manager/launch/gnss_imu_gt_ctra.launch
base_ws/src/bringup/launch/localization_gnss_imu_gt_ctra.launch
configs/localization/gnss_imu_gt_noise_sensor6_ideal.yaml
configs/localization/robot_localization_gnss_imu_gt_ctra_local.yaml
configs/localization/robot_localization_gnss_imu_gt_ctra_global.yaml
```

GT local/global YAML은 기존 EKF의 frequency, process noise와 초기
covariance를 재사용하는 overlay다. launch에서 기존 YAML을 먼저
로드하고 GT YAML을 나중에 로드한다.

## 11. MORAI 실시간 실행

터미널 1:

```bash
# ROS master 실행
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roscore
```

터미널 2:

```bash
# MORAI UDP 센서 수신
cd ~/AjouNice2026/base_ws
source devel/setup.bash

roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:=/home/luv/AjouNice2026/configs/sensors/Sensor.json \
  network_config:=/home/luv/AjouNice2026/configs/network/ego_network_1.json
```

터미널 3:

```bash
# 현재 MORAI에서 Ego를 GT 대체 입력으로 사용해 CTRA 측위 실행
cd ~/AjouNice2026/base_ws
source devel/setup.bash

roslaunch bringup localization_gnss_imu_gt_ctra.launch \
  use_sim_time:=false \
  gt_source:=ego \
  publish_sensor_tf:=true
```

실제 Competition 입력으로 전환:

```bash
# 대회 제공 heading/vx 토픽으로 전환
cd ~/AjouNice2026/base_ws
source devel/setup.bash

roslaunch bringup localization_gnss_imu_gt_ctra.launch \
  use_sim_time:=false \
  gt_source:=competition \
  publish_sensor_tf:=true
```

이미 `sensing.launch`가 sensor TF를 발행 중이면 중복을 막기 위해
`publish_sensor_tf:=false`로 실행한다.

출력 확인:

```bash
# GT adapter와 최종 localization 출력 주기 확인
rostopic hz --wall-time -w 200 /localization/gt/orientation
rostopic hz --wall-time -w 200 /localization/gt/twist
rostopic hz --wall-time -w 200 /localization/kinematic_state
rostopic hz --wall-time -w 200 /localization/acceleration
```

## 12. rosbag 기록

기록 스크립트는 Ego와 Competition을 모두 포함한다.

```bash
# GT CTRA 입력·중간 토픽·출력 기록
cd ~/AjouNice2026
source base_ws/devel/setup.bash

GNSS_IMU_EVAL_BAG="$PWD/rosbags/eval/localization/gnss_imu_gt_ctra_run01.bag" \
  scripts/rosbag/record/rosbag_for_gnss_imu_eval.sh
```

## 13. 기존 rosbag 30초 재생

대상:

```text
rosbags/raw_sensor/competition_route_full_highspeed.bag
```

메타데이터 기준:

| 입력 | 평균 도착률 |
|---|---:|
| GPS | 약 28.2 Hz |
| IMU | 약 45.2 Hz |
| Ego | 약 48.5 Hz |

터미널 1:

```bash
# rosbag 시간으로 GT CTRA 측위 실행
cd ~/AjouNice2026/base_ws
source devel/setup.bash

roslaunch bringup localization_gnss_imu_gt_ctra.launch \
  use_sim_time:=true \
  gt_source:=ego \
  publish_sensor_tf:=true
```

터미널 2:

```bash
# 고속 주행 bag의 5초 지점부터 30초만 재생
cd ~/AjouNice2026
source base_ws/devel/setup.bash

rosbag play rosbags/raw_sensor/competition_route_full_highspeed.bag \
  --clock \
  --start 5 \
  --duration 30
```

필요한 경우 터미널 3에서 위 기록 스크립트를 실행한다.

## 14. Competition source 경로만 빠르게 시험

기존 bag의 Ego를 Competition 토픽으로 임시 relay한다.

```bash
# 기존 Ego 메시지를 Competition 입력 경로로 전달
source ~/AjouNice2026/base_ws/devel/setup.bash
rosrun topic_tools relay /Ego_topic /Competition_topic
```

이 상태에서 launch를 `gt_source:=competition`으로 실행하면 source 선택,
구독 토픽, 메시지 변환과 EKF 연결을 확인할 수 있다. 이는 두 값이 실제
대회에서도 같다는 성능 증거가 아니라 통신 경로 검증이다.

## 15. 평가

기록한 결과 bag을 기존 평가기로 확인한다.

```bash
# GT CTRA localization을 Ego 기준으로 평가
cd ~/AjouNice2026
source base_ws/devel/setup.bash

python3 scripts/evaluation/localization/eval_localization_state_with_rviz.py \
  rosbags/eval/localization/gnss_imu_gt_ctra_run01.bag \
  --state-topic /localization/kinematic_state \
  --sensor-config configs/sensors/Sensor.json \
  --ego-velocity-unit mps \
  --alignment-reference-sec 0.0 \
  --out rosbags/eval/localization/gnss_imu_gt_ctra_run01.csv \
  --position-error-threshold 1.0 \
  --speed-error-threshold 0.5 \
  --yaw-rate-error-threshold 0.1
```

GT heading과 velocity를 입력으로 사용하면서 같은 Ego를 정답으로
비교하므로 yaw와 speed 지표는 독립적인 알고리즘 성능이 아니다. 다음
오류를 찾는 smoke/regression 검사로 해석한다.

- degree/radian 오류
- km/h/m/s 오류
- heading 축 및 부호 오류
- timestamp 중복·역행
- frame 변환 오류
- 초기화 spike

## 16. 260801 최소 smoke test 결과

`competition_route_full_highspeed.bag`의 5초 지점부터 20초를
`gt_source:=ego`로 재생했다.

| 토픽 | 메시지 수 | timestamp 기준 주기 |
|---|---:|---:|
| `/localization/gt/orientation` | 810 | 40.466 Hz |
| `/localization/gt/twist` | 810 | 40.466 Hz |
| `/localization/imu` | 661 | 33.061 Hz |
| `/localization/gnss/odom` | 561 | 28.052 Hz |
| `/localization/odometry/local` | 999 | 49.999 Hz |
| `/localization/odom` | 998 | 49.999 Hz |
| `/localization/kinematic_state` | 998 | 49.999 Hz |

모든 위 토픽에서 다음 조건을 만족했다.

```text
non-increasing timestamp: 0
NaN/Inf: 0
최종 parent frame: map
최종 child frame: base_link
```

동일 구간의 smoke 평가:

| 지표 | 결과 |
|---|---:|
| XY RMSE | 0.239 m |
| XY 최대 오차 | 0.459 m |
| yaw MAE | 0.064 deg |
| speed MAE | 0.068 m/s |
| 종방향 MAE | 0.192 m |
| 횡방향 MAE | 0.078 m |

재생 시작 시 차량이 이미 약 7.5 m/s로 주행 중이어서 EKF가 첫 GT
속도를 받기 전 한 샘플에 7.504 m/s 속도 오차가 발생했다. 초기 output
readiness gate는 기존 결정대로 이번 구현에 추가하지 않았다. 실제
정지 출발에서는 영향이 작지만 주행 중 launch를 다시 시작할 때는 별도
대책이 필요하다.

global EKF는 `transform_timeout: 0.0` 정책에 따라 같은 시각의 local TF가
아직 없으면 latest `odom -> base_link`를 사용한다는 경고를 냈지만,
20초 동안 최종 state 누락, 비정상 frame, timestamp 역행은 없었다.

추가로 Ego 입력을 rosbag remap으로 `/Competition_topic`에 전달하고
`gt_source:=competition`으로 5초 실행했다.

```text
/Competition_topic:              238 messages
/localization/gt/orientation:    188 messages
/localization/gt/twist:          188 messages
/localization/kinematic_state:   249 messages
fatal/error:                     0
```

따라서 source 선택과 Competition 통신 경로는 동작한다. 실제 대회
Competition 데이터의 값과 단위 검증은 별도로 남아 있다.

## 17. 기존 GNSS+IMU와 동일 구간 비교

기존 GNSS+IMU와 GT CTRA를 같은 bag, 같은 5~25초 구간으로 각각
실행했다. 이 구간은 차량이 이미 주행 중인 상태에서 시작하므로 두
결과 모두 첫 2초를 제외하고 먼저 비교했다. 다만 기존 GNSS+IMU는
직접 속도 입력이 없어 2초 이후에도 초기 속도 수렴이 계속되므로 이
표는 정상상태 정확도뿐 아니라 startup 복구 성능도 포함한다.

| 지표 | 기존 GNSS+IMU | GT CTRA | 변화 |
|---|---:|---:|---:|
| XY mean | 0.380 m | 0.082 m | 78.4% 감소 |
| XY RMSE | 0.405 m | 0.094 m | 76.8% 감소 |
| XY max | 0.854 m | 0.298 m | 65.1% 감소 |
| drift | 0.269% | 0.083% | 69.1% 감소 |
| 종방향 MAE | 0.286 m | 0.064 m | 77.6% 감소 |
| 횡방향 MAE | 0.167 m | 0.039 m | 76.6% 감소 |
| yaw MAE | 0.120 deg | 0.072 deg | 40.0% 감소 |
| speed MAE | 0.715 m/s | 0.037 m/s | 94.8% 감소 |
| speed RMSE | 1.102 m/s | 0.070 m/s | 93.6% 감소 |
| yaw-rate MAE | 0.0155 rad/s | 0.0147 rad/s | 유사 |

출력 주기:

| 항목 | 기존 GNSS+IMU | GT CTRA |
|---|---:|---:|
| 평균 state Hz | 48.268 Hz | 49.999 Hz |
| 간격 표준편차 | 7.598 ms | 1.780 ms |
| 최대 간격 | 146.969 ms | 31.729 ms |

GT CTRA의 position과 speed 개선은 예상된 결과다. 같은 Ego heading과
velocity를 GT CTRA의 입력과 평가 정답으로 함께 사용했기 때문에 이
수치를 독립적인 센서융합 정확도 향상으로 해석하면 안 된다. 대신
GT 값을 올바른 frame, 단위, timestamp로 EKF에 연결했는지와 GNSS 위치
출력이 안정적으로 따라오는지를 확인하는 회귀 지표다.

출력 주기 차이 역시 한 번의 짧은 실행 결과이므로 GT 입력 자체가
스케줄링을 항상 개선한다고 일반화하지 않는다. 장시간 live 실행에서
주기 분포를 다시 측정해야 한다.

초기 5초를 완전히 제외한 정상상태에 가까운 비교는 다음과 같다.

| 지표 | 기존 GNSS+IMU | GT CTRA |
|---|---:|---:|
| XY mean | 0.184 m | 0.128 m |
| XY RMSE | 0.218 m | 0.139 m |
| 종방향 MAE | 0.078 m | 0.081 m |
| 횡방향 MAE | 0.149 m | 0.082 m |
| yaw MAE | 0.147 deg | 0.086 deg |
| longitudinal vx MAE | 0.067 m/s | 0.009 m/s |
| 3D speed MAE | 0.578 m/s | 0.020 m/s |

정상상태에서는 앞선 표보다 위치 차이가 작아진다. GT CTRA의 주된
차이는 직접 vx 입력으로 초기 수렴이 빠르고, 기존 baseline에서 IMU
`ay`를 적분해 생긴 불필요한 횡속도가 제거됐다는 점이다. 종방향 위치
MAE는 같은 수준이고 횡방향·yaw·속도 안정성이 개선됐다.

## 18. 정지 초기화부터 GNSS 음영 직전까지 공정 비교

`competition_route_full_highspeed.bag`의 timestamp를 검사한 결과:

```text
차량 움직이기 시작: 3.293 sec
GNSS (0,0,0) 시작: 132.850 sec
GNSS 정상 복구: 139.966 sec
```

두 provider를 각각 새 ROS master에서 `--start 0 --duration 132.5`,
1배속으로 실행했다. 첫 3.5초는 EKF 초기화에 사용하고 평가 지표에서는
제외했다. 평가 주행 시간은 약 128.95초, Ego 경로는 약 1,876.7 m다.

| 지표 | 기존 GNSS+IMU | GT CTRA | 판단 |
|---|---:|---:|---|
| XY mean | 0.162 m | 0.138 m | GT 14.8% 감소 |
| XY RMSE | 0.198 m | 0.169 m | GT 14.6% 감소 |
| XY max | 0.765 m | 0.778 m | 사실상 동일 |
| XY final | 0.209 m | 0.063 m | GT 감소 |
| drift | 0.011% | 0.003% | 둘 다 매우 작음 |
| 종방향 MAE | 0.115 m | 0.121 m | baseline이 근소하게 작음 |
| 횡방향 MAE | 0.086 m | 0.046 m | GT 46.5% 감소 |
| yaw MAE | 0.074 deg | 0.040 deg | GT 45.9% 감소 |
| yaw max | 2.498 deg | 0.861 deg | GT 감소 |
| longitudinal vx MAE | 0.069 m/s | 0.009 m/s | GT 직접 입력 효과 |
| speed MAE | 0.319 m/s | 0.014 m/s | GT 직접 입력 효과 |
| yaw-rate MAE | 0.0089 rad/s | 0.0089 rad/s | 동일 IMU 입력으로 동급 |
| 추정 경로 길이 오차 | +46.448 m | +9.944 m | GT의 경로 떨림 감소 |

출력 주기:

| 항목 | 기존 GNSS+IMU | GT CTRA |
|---|---:|---:|
| state 수 | 6,624 | 6,624 |
| 평균 Hz | 49.999998 | 49.996795 |
| 간격 표준편차 | 1.266 ms | 1.679 ms |
| 최대 간격 | 30.242 ms | 30.325 ms |
| 비증가 timestamp | 0 | 0 |

따라서 `frequency: 50`, `predict_to_current_time: true`의 출력 주기는 두
provider 모두 정상이며 GT가 주기를 개선했다고 볼 근거는 없다.

GT CTRA의 위치 평균 개선은 약 15%로 제한적이다. noise-off GNSS가 이미
절대 위치를 강하게 보정하므로 예상되는 결과다. 주된 이득은 횡방향
안정성, yaw, 속도, 추정 경로 길이에서 나타났다. 종방향 위치 오차는
동급이다.

GT CTRA speed/yaw는 같은 Ego를 입력과 평가 정답에 함께 사용하므로
독립적인 알고리즘 성능 지표가 아니다. 메시지 단위·frame·timestamp와
EKF 연결이 정상임을 확인하는 통합 검증으로 해석한다.

## 19. 현재 한계와 후속 작업

- 실제 `/Competition_topic` rosbag으로 검증되지 않았다.
- GT `velocity.x`가 차량 종방향 m/s라는 계약을 대회 환경에서 다시
  확인해야 한다.
- `vy ≈ 0` 가정이므로 큰 sideslip은 모델링하지 않는다.
- GNSS가 `(0, 0, 0)`으로 들어오는 음영 구간은 GNSS adapter에서
  차단하며, 이 구간에는 GT motion과 IMU로 상태를 예측한다.
- 실제 noise 설정 후 covariance를 다시 추정해야 한다.
- 공식 시작 pose 인터페이스가 확정되면 첫 Ego 초기화를 manual 시작
  pose로 교체한다.

## 20. MORAI 시나리오 heading 보정

대회 제공 시나리오의 시작 pose는 yaw `62.515 deg`이다. 같은 위치
`(-131.485992, -427.960999)`에서 정지한 MORAI `/Ego_topic` heading은
`63.862 deg`로 관측되어, 시나리오 기준 차이는 `+1.347 deg`였다.

GT CTRA provider는 시나리오 좌표계와 맞추기 위해 기본적으로 다음 보정을
초기 pose와 연속 GT heading 모두에 적용한다.

```text
corrected_heading = raw_heading - 1.347 deg
```

설정은 `configs/localization/localization_manager.yaml`의
`providers/gnss_imu_gt` 아래에 기록되어 있으며, launch 인자로 즉시
끄거나 값을 바꿀 수 있다.

```bash
# 시나리오 보정 활성화: 현재 기본값
roslaunch bringup localization_gnss_imu_gt_ctra.launch \
  use_heading_correction:=true \
  heading_yaw_offset_deg:=-1.347

# MORAI heading 문제가 수정된 환경
roslaunch bringup localization_gnss_imu_gt_ctra.launch \
  use_heading_correction:=false
```

이 보정은 개발 중 첫 Ego pose를 사용하는 경우에만 초기 pose에도
적용된다. 공식 시작 pose를 `manual`로 직접 입력할 때는 이미 시나리오
좌표계 값이므로 추가 보정하지 않는다.

평가 시에도 정답 `/Ego_topic` heading에 같은 보정을 적용해야 yaw,
종·횡방향 오차 축이 동일해진다.

```bash
python3 scripts/evaluation/localization/eval_localization_state_with_rviz.py \
  INPUT.bag \
  --state-topic /localization/kinematic_state \
  --ego-heading-yaw-offset-deg -1.347 \
  --alignment-yaw-mode none
```

`--alignment-yaw-mode none`은 평가기가 시작 pose에서 yaw 차이를 사후
회전하여 없애지 않고 위치 원점만 맞춘다. 따라서 시나리오 기준의 절대
heading 보정 전후를 비교할 때 사용한다. 시작 방향이 임의적인 로컬
좌표계의 궤적을 비교할 때만 기존 기본값 `estimate`를 사용한다.

`-1.347 deg`는 현재 제공 시나리오에 대한 보정값이다. 대회 시뮬레이터가
수정되거나 공식 좌표계 계약이 달라지면 보정을 끄고 다시 검증한다.

## 21. 최종 rosbag RViz 시각화

최종 결과 bag은 localization, TF, IMU, GNSS만 담은 경량 bag이다.
LiDAR와 카메라는 같은 시작·종료 timestamp를 가진 원본 bag에서 함께
재생한다.

```text
원본 센서:
  rosbags/raw_sensor/competition_route_full_highspeed.bag

최종 localization:
  rosbags/eval/localization/
    gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3/
      gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3.bag
```

### 터미널 1: ROS master

```bash
# ROS master 실행
cd ~/AjouNice2026/base_ws
source devel/setup.bash

roscore
```

### 터미널 2: 지도·차량·센서 TF·Global Minimap·RViz

통합 RViz에서는 MGeo link 중심선만 기본 발행한다. 폭 기반 추정 경계,
노드, 신호, 원점 축 및 메타데이터는 기본적으로 발행하지 않는다.

```bash
# Lanelet·MGeo 중심선·차량·센서 TF·2D minimap과 통합 RViz 실행
cd ~/AjouNice2026/base_ws
source devel/setup.bash

rosparam set /use_sim_time true

roslaunch localization_manager \
  gnss_imu_lanelet_rviz.launch \
  lanelet_map_file:=/home/luv/AjouNice2026/kcity_map/lanelet_map/lanelet2_map_v60.osm \
  show_minimap:=true
```

`lanelet2_map_v60.osm`은 OSM 내부 위·경도와 기존
`map_projector_info_v1.yaml`을 사용하므로 별도의 JSON 메타데이터가
필요하지 않다. 현재 파일이 Git에서 추적되지 않는 로컬 전달본일 수
있으므로, 팀 저장소에 포함되기 전까지는 위와 같이 경로를 명시한다.

이 launch는 기본적으로 `/localization/kinematic_state`를 구독하는
`localization_state_rviz_marker.py`도 실행한다. RViz의
`Vehicle Speed and Twist` 표시에는 다음 정보가 나타난다.

- 차량 위 흰색 글자: 현재 수평 속력(m/s, km/h), `vx`, `vy`, yaw rate,
  state 도착률, 메시지 age
- 청록색 화살표: `base_link` 기준 수평 속도 방향과 크기

이 표시는 측위 결과를 변경하지 않는 디버그 시각화다. 필요하면 launch에
`show_state_overlay:=false`를 추가하여 끌 수 있다.

동일 launch는 기본적으로 `localization_minimap_publisher.py`도 실행한다.
이 노드는 `/map/lanelet_map`과 `/localization/kinematic_state`로
North-up 전역 지도를 그려 `/localization/minimap/image`에 `bgr8`
이미지를 `10 Hz`로 발행한다. RViz의 `Global Minimap` Image display에는
전체 Lanelet, 현재 차량 위치·heading·속도 및 재생 시작 이후 누적
localization 경로가 나타난다.

minimap은 rosbag에 저장된 영상이 아니라 기존 state를 이용해 재생 중
생성되므로 rosbag topic 목록을 변경하지 않는다. 필요하면 launch에
`show_minimap:=false`를 추가하여 끌 수 있다.

선택적으로 `show_mgeo_width_edges:=true`를 주면 나타나는 파란 양쪽
선의 namespace는
`mgeo_inferred_width_edges`이다. 이는 MGeo가 제공한 실제 좌·우 경계
좌표가 아니라 `link_set.json`의 중심선을 `width_start`/`width_end`의
절반만큼 좌우로 이동하여 추정한 선이다. 따라서 LiDAR intensity에서
관찰되는 차선 도료보다 좁거나 넓을 수 있으며 정밀 정합의 기준선으로
사용하면 안 된다.

이 launch는 시각화에만 다음 보정을 적용한다.

```text
Lanelet 원본 고도 -> 평탄화
Lanelet 기준면    -> map z=-0.3705 m
MGeo 원본 고도    -> 평탄화
MGeo 기준면       -> morai_map z=-0.3705 m
```

Localization 결과, 원본 지도, 차량 URDF 및 판단용 vector map은 수정하지
않는다.

### 터미널 3: 전체 GT·추정 Path 발행

Lanelet 중심선은 기준면보다 `0.06 m` 높게 표시되므로 Path 표시 높이는
`-0.3705 + 0.06 = -0.3105 m`로 맞춘다. 이 값은 RViz에 발행하는
Path/Marker에만 적용되며 평가 CSV와 지표는 변경하지 않는다.

```bash
# Lanelet 중심선과 같은 높이에 GT·Localization 전체 Path 발행
cd ~/AjouNice2026
source base_ws/devel/setup.bash

python3 scripts/evaluation/localization/eval_localization_state_with_rviz.py \
  rosbags/eval/localization/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3.bag \
  --state-topic /localization/kinematic_state \
  --sensor-config configs/sensors/Sensor.json \
  --ego-velocity-unit mps \
  --ego-heading-yaw-offset-deg -1.347 \
  --alignment-reference-sec 0.0 \
  --alignment-yaw-mode none \
  --rviz-ground-z -0.3105 \
  --position-error-threshold 1.0 \
  --speed-error-threshold 0.5 \
  --yaw-rate-error-threshold 0.1 \
  --publish-rviz
```

이 프로세스는 latched Path와 Marker를 유지하므로 시각화가 끝날 때까지
종료하지 않는다.

### 터미널 4: 센서 원본과 localization 결과 병합 재생

```bash
# 동일 시간축의 카메라·LiDAR 원본과 Localization 결과 동시 재생
cd ~/AjouNice2026/base_ws
source devel/setup.bash

rosbag play --clock --pause \
  /home/luv/AjouNice2026/rosbags/raw_sensor/competition_route_full_highspeed.bag \
  /home/luv/AjouNice2026/rosbags/eval/localization/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3/gnss_imu_gt_ctra_heading_corrected_vx_ax_tuned_full_v3.bag \
  --topics \
  /sensor/lidar1 \
  /sensor/camera/front/compressed \
  /sensor/camera/front_left/compressed \
  /sensor/camera/front_right/compressed \
  /sensor/camera/back/compressed \
  /localization/kinematic_state \
  /localization/acceleration \
  /tf \
  /tf_static
```

`rosbag play` 터미널에서 스페이스바를 눌러 재생한다. 일시정지 상태에서는
`map -> morai_map` 등의 recorded TF가 아직 발행되지 않아 RViz가 일시적인
TF 경고를 표시할 수 있다.

### RViz Display

```text
Fixed Frame: map

Global Minimap
  /localization/minimap/image + raw transport

Vehicle
  IONIQ5 RobotModel

LiDAR1
  /sensor/lidar1

Front Camera
  /sensor/camera/front + compressed transport

Front Left Camera
  /sensor/camera/front_left + compressed transport

Front Right Camera
  /sensor/camera/front_right + compressed transport

Rear Camera
  /sensor/camera/back + compressed transport

Localization Odometry
  /localization/kinematic_state

MORAI GT path
  /localization_state_eval/ego_path

GNSS IMU estimated path
  /localization_state_eval/state_aligned_path

Threshold violations
  /localization_state_eval/error_markers

Lanelet Map
  /map/lanelet_map

MGeo Overlay
  /map/morai_map
```

GT와 추정 Path는 전체 경로를 한 번에 표시하며 각각 독립적으로 끄고 켤
수 있다. `Localization Odometry`는 rosbag 재생에 따라 진행 경로가
순차적으로 쌓인다.

`Threshold violations`는 임계값 초과 지점만 표시한다. 초과 지점이 없는
종류는 `Marker.DELETE`로 발행하여 빈 `SPHERE_LIST` RViz 오류가 발생하지
않는다.

MGeo width edge는 `link centerline +/- width/2`로 계산한 추정 경계이며
원본 경계 geometry가 아니다. LiDAR 노면 반사와 비교할 때 이 한계를
전제로 사용한다.

## 지도와 LiDAR 노면 도료의 미세 정합 검토

> 이 절은 Lanelet v60 시점의 과거 진단 기록이다. 현재 v61 런타임은
> `map -> morai_map = (131.559130220, 545.911338186, 0.0)`, `yaw=0`을
> 사용하며, 아래의 기존·반대 미세 보정값은 적용하지 않는다.

### 관찰

Lanelet v60 중심선과 MGeo 중심선은 미세 보정 전 `map -> morai_map`
변환에서 육안상 매우 잘 정합되었다. 반면 LiDAR intensity에서 밝게
관찰되는 노면 도료는 Lanelet 도로 경계보다 약간 오른쪽에 있는 것처럼
보였다. 제한된 전방 구간만 보면 이 현상은 평행이동인지 작은 회전인지
구분하기 어려웠다.

검토한 `map -> morai_map` 값은 다음과 같다.

| 구분 | x (m) | y (m) | yaw |
|---|---:|---:|---:|
| 미세 보정 전 원본 | 131.559130220 | 545.911338186 | 0 rad |
| 기존 미세 보정 | 131.656525062 | 545.893929229 | +0.000146070411 rad |
| 진단용 반대 보정 | 131.461737922 | 545.928761369 | -0.000146070411 rad |

반대 보정은 기존 보정의 부호만 단순히 바꾼 근삿값이 아니라, 원본
변환을 기준으로 기존 SE(2) 미세 변환의 역변환을 합성한 진단값이다.
기존 보정과 반대 보정을 비교해 어느 방향으로 표시가 움직이는지
확인하는 용도로만 사용하며 최종 보정값으로 채택하지 않는다.

### MORAI heading 오차와의 관계

지도 중심선끼리의 정적 정합에는 `/Ego_topic` heading이 사용되지
않으므로, MGeo와 Lanelet 자체의 차이는 기존 `+1.347 deg` heading
편차로 설명할 수 없다.

다만 LiDAR 점군을 `map`에서 볼 때는 `base_link` 자세가 TF 체인에
포함되므로, 보정되지 않은 차량 yaw는 노면 도료가 지도에 대해 회전한
것처럼 보이게 할 수 있다. `1.347 deg`의 회전은 차량 전방 거리별로
대략 다음 횡방향 편차를 만든다.

| 전방 거리 | 예상 횡방향 편차 |
|---:|---:|
| 5 m | 약 0.12 m |
| 10 m | 약 0.24 m |
| 20 m | 약 0.47 m |
| 30 m | 약 0.71 m |

따라서 과거 보정 전 TF로 LiDAR를 관찰했다면 heading 편차가 원인일
가능성이 있다. 그러나 현재 사용하는
`gnss_imu_gt_ctra_heading_corrected_full_v1.bag`은 GT heading에
`-1.347 deg`를 적용해 생성했으며, 해당 결과의 yaw MAE는 약
`0.035 deg`이다. 현재 bag에서도 편차가 남는다면 heading 하나만으로
설명하기 어렵고 다음 항목을 함께 확인해야 한다.

- 보정된 localization TF와 별도 Ego TF broadcaster의 중복 발행 여부
- `base_link -> lidar1` translation/yaw extrinsic
- LiDAR timestamp와 동적 TF timestamp 정합
- Lanelet v60과 MGeo 데이터 사이의 실제 국부 형상 차이
- 기존 미세 보정이 Lanelet v23 기준으로 산출됐다는 지도 버전 차이

### 현재 결정

지도 자체가 잘 맞는 상태에서 LiDAR 한 종류의 관찰만으로
`map -> morai_map`을 이동시키면, 특정 위치의 도료는 맞더라도 두 지도
전체 정합을 훼손할 수 있다. 따라서 이번 검토에서는 기존 미세 보정과
반대 보정 모두 최종값으로 새로 확정하지 않는다. 향후에는 미세 보정 전
TF를 기준으로 다음 A/B 검증을 수행한다.

1. 차량과 rosbag 없이 Lanelet 중심선과 MGeo 중심선만 비교한다.
2. 동일한 원본 센서 bag에 heading 보정 전·후 TF를 각각 적용한다.
3. 직선 구간에서 가까운 점과 먼 점의 편차가 일정한지 확인한다.
4. 일정하면 translation/extrinsic을, 거리에 비례하면 yaw를 우선
   의심한다.
5. 서로 다른 차량 진행 방향에서도 같은 결과가 재현되는지 확인한다.

수동 `map -> morai_map` TF를 시험할 때는 결과 bag에 저장된
`/tf_static`을 재생하면 안 된다. recorded TF와 수동 TF가 동시에 같은
child frame을 발행하여 RViz 결과가 불안정해질 수 있기 때문이다.
