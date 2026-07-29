# 260725 GNSS + IMU `robot_localization` 구현

## 1. 목적

기존 LiDAR 기반 측위와 별개로, MORAI GNSS와 IMU만 사용하는 가벼운
localization baseline을 추가했다.

목표는 다음과 같다.

- 기존 FAST-LIO 및 FAST-LIO+GNSS 코드를 변경하지 않는다.
- Planning에는 기존과 동일한
  `/localization/kinematic_state` (`nav_msgs/Odometry`)를 제공한다.
- MORAI 전용 메시지 변환과 공분산 부여를 fusion backend에서 분리한다.
- `robot_localization`을 나중에 다른 fusion backend로 교체하더라도 센서
  전처리와 Planning 출력 계약은 재사용할 수 있게 한다.
- 처음에는 노이즈가 꺼진 `Sensor_4.json`으로 기준 성능을 측정하고,
  그 다음 `Sensor_4_noise_v1.json`으로 노이즈 실험을 수행한다.

이번 구현은 IMU fixed bias 영점 조정을 포함하지 않는다. 현재 MORAI 설정은
zero-mean noise만 제공하고 fixed bias를 설정하지 않았기 때문이다.

## 2. 전체 데이터 흐름

```text
MORAI / rosbag
├─ first /Ego_topic pose (development start-pose proxy)
│    │
│    └─ gnss_imu_initial_pose_publisher.py
│         - source: ego | manual
│         - morai_map의 시작 pose를 map pose로 변환
│         - global EKF에만 set_pose
│         └─ /localization/initialpose
│
├─ /sensor/imu  (sensor_msgs/Imu)
│    │
│    └─ imu_covariance_adapter.py
│         - 측정값은 변경하지 않음
│         - quaternion 정규화
│         - 중복 timestamp 제거
│         - -1/0/비정상 covariance를 YAML 값으로 대체
│              │
│              └─ /localization/imu (sensor_msgs/Imu)
│
└─ /sensor/gps  (morai_msgs/GPSMessage, WGS84)
     │
     └─ morai_gps_to_local_odometry.py
          - 위도/경도 → UTM 52N
          - message offset이 유효하면 사용
          - 현재 bag처럼 offset=0이면 K-City fallback offset 사용
          - morai_map 좌표 → map 좌표로 변환
          - Sensor_4.json의 GPS 높이 보정
          - GNSS 위치 covariance 부여
               │
               └─ /localization/gnss/odom (nav_msgs/Odometry)

/localization/imu
    ├─ local EKF (world_frame=odom)
    │    ├─ /localization/odometry/local
    │    └─ TF odom → base_link
    │
    └─ global EKF (world_frame=map)
         + /localization/gnss/odom의 x, y
         ├─ /localization/odom
         └─ TF map → odom

/localization/odom
    └─ gnss_imu_kinematic_state_publisher.py
         - pose/twist/covariance 복사
         - TF는 발행하지 않음
         └─ /localization/kinematic_state
```

## 3. 왜 두 개의 EKF를 사용하는가

REP-105 형태의 TF를 유지하기 위해 역할을 나눴다.

- local EKF
  - IMU dead reckoning을 수행한다.
  - 연속적인 `odom → base_link`를 소유한다.
- global EKF
  - IMU와 GNSS 위치를 함께 사용한다.
  - GNSS의 절대 위치 보정을 `map → odom`으로 표현한다.

따라서 최종 TF tree는 다음과 같다.

```text
map → odom → base_link → imu
                       └→ gps
map → morai_map
```

두 EKF가 같은 TF edge를 중복 발행하지 않는다. 최종
`gnss_imu_kinematic_state_publisher.py`도 TF를 발행하지 않는다.

`transform_timeout`은 `0.0`으로 설정했다. global EKF가 매 주기마다 아직
도착하지 않은 정확한 시각의 `odom → base_link`를 50 ms씩 기다리면 bag
재생에서 불필요한 blocking과 update-rate warning이 반복됐기 때문이다.
global EKF는 local EKF의 최신 TF를 즉시 사용한다. global EKF의
`silent_tf_failure`는 이 의도된 latest-TF fallback warning만 숨기며 실제
measurement/estimator 오류를 숨기는 용도가 아니다.

## 4. 초기 pose 정책

FAST-LIO+GNSS와 동일하게 개발 환경의 첫 `/Ego_topic`을 대회에서 제공될
시작 pose의 대용으로 사용한다.

```text
개발:
첫 /Ego_topic pose
→ morai_map에서 map으로 변환
→ /localization/initialpose
→ global EKF set_pose

대회:
제공된 시작 pose
→ providers/gnss_imu/initialization/manual
→ 같은 /localization/initialpose 경로 재사용
```

설정은 `configs/localization/localization_manager.yaml`에 있다.

```yaml
providers:
  gnss_imu:
    initialization:
      source: ego  # ego | manual
      heading_unit: degrees
      manual:
        frame_id: morai_map
        position: [0.0, 0.0, 0.0]
        rpy_degrees: [0.0, 0.0, 0.0]
```

초기 pose는 global EKF에만 적용한다. local EKF는 출발 지점을 상대좌표
`odom`의 원점으로 유지해야 하므로 전역 map pose로 reset하지 않는다.

이 변경 전에는 IMU orientation이 global EKF를 먼저 초기화한 직후 첫
GNSS 절대위치가 들어오면서, 정지 상태에서도 순간적인 속도 추정이
발생했다. 초기 pose를 먼저 설정하면 첫 GNSS residual이 작아져 이
startup spike가 크게 감소한다.

이 초기화는 출력 gate가 아니다. global EKF와
`/localization/kinematic_state`는 초기화 직후부터 계속 발행한다.
Planning/Control에 전달하기 전 안정 여부를 판정하는 readiness gate는
요청에 따라 이번 변경에서 보류했다.

## 5. 왜 `/localization/gnss/fix`를 사용하지 않는가

이번 provider는 아래의 기존 FAST-LIO+GNSS 경로를 수정하지 않는다.

```text
/sensor/gps
→ morai_gps_to_navsatfix.py
→ /localization/gnss/fix (sensor_msgs/NavSatFix)
→ FAST-LIO MSF 내부 GNSS 처리
```

GNSS+IMU provider에서는 MORAI의 K-City offset과 이미 존재하는
`map ↔ morai_map` 관계를 명시적으로 알고 있으므로, 다음처럼 직접 위치
측정 Odometry를 만든다.

```text
/sensor/gps
→ morai_gps_to_local_odometry.py
→ /localization/gnss/odom
→ robot_localization
```

이 방식은 `navsat_transform_node`를 반드시 거쳐야 하는 일반 하드웨어
구성과 달리, MORAI 전용 전처리 조건을 한 파일에 격리한다. fusion
backend를 바꾸더라도 `/localization/imu`와 `/localization/gnss/odom`은
재사용할 수 있다.

## 6. covariance 설계

MORAI의 `/sensor/imu` 메시지는 현재 covariance 첫 원소를 `-1`로
발행한다. 이것은 “covariance를 알 수 없음”이라는 ROS 표현이지, 실제
분산이 음수라는 뜻이 아니다. `/sensor/gps`의 `GPSMessage`에는 covariance
필드 자체가 없다.

그래서 UDP receiver는 수정하지 않고 별도 전처리 노드가 다음 값을
부여한다.

### 노이즈 OFF 기준값

파일:
`configs/localization/gnss_imu_noise_sensor4_ideal.yaml`

| 측정 | 표준편차 | 메시지에 들어가는 분산 |
|---|---:|---:|
| IMU orientation | 0.001745 rad (0.1°) | 3.046e-6 |
| IMU angular velocity | 0.001 rad/s | 1.0e-6 |
| IMU linear acceleration | 0.01 m/s² | 1.0e-4 |
| GNSS x/y | 0.05 m | 0.0025 |
| GNSS z | 1.0 m | 1.0 |

노이즈가 꺼져 있어도 covariance를 정확히 0으로 두지 않았다. 0은 EKF가
센서를 수학적으로 완벽하다고 과신하게 만들며 수치적으로도 불리하다.
위 값은 현재의 작은 baseline uncertainty이지 최종 대회용 보정값은 아니다.

### 노이즈 ON 초안

- 센서: `configs/sensors/Sensor_4_noise_v1.json`
- uncertainty:
  `configs/localization/gnss_imu_noise_sensor4_noise_v1.yaml`

`Sensor_4_noise_v1.json`은 Sensor_4의 extrinsic과 발행 주기를 그대로
유지하며 다음 noise만 활성화한다.

- GPS Gaussian noise: standard deviation 0.3 m
- IMU acceleration white noise x/y/z: standard deviation 0.001
- IMU angular-velocity white noise roll/pitch/yaw: standard deviation 0.001

noise-on covariance YAML은 첫 실험을 위한 초안이다. 여러 번 녹화한 뒤
GT residual의 분산으로 다시 추정해야 하며, 현재 값을 최종 calibration으로
간주하면 안 된다.

## 7. 변경 파일

### 추가

- `base_ws/src/bringup/launch/localization_gnss_imu.launch`
- `base_ws/src/localization/localization_manager/launch/gnss_imu.launch`
- `base_ws/src/localization/localization_manager/scripts/imu_covariance_adapter.py`
- `base_ws/src/localization/localization_manager/scripts/morai_gps_to_local_odometry.py`
- `base_ws/src/localization/localization_manager/scripts/gnss_imu_initial_pose_publisher.py`
- `base_ws/src/localization/localization_manager/scripts/gnss_imu_kinematic_state_publisher.py`
- `base_ws/src/localization/localization_manager/rviz/gnss_imu_evaluation.rviz`
- `configs/localization/gnss_imu_noise_sensor4_ideal.yaml`
- `configs/localization/gnss_imu_noise_sensor4_noise_v1.yaml`
- `configs/localization/robot_localization_gnss_imu_local.yaml`
- `configs/localization/robot_localization_gnss_imu_global.yaml`
- `configs/sensors/Sensor_4_noise_v1.json`
- `scripts/rosbag/record/rosbag_for_gnss_imu_eval.sh`
- 이 문서와 별도 평가 결과 문서

### 수정

- `base_ws/src/localization/localization_manager/CMakeLists.txt`
- `base_ws/src/localization/localization_manager/package.xml`
- `base_ws/src/localization/localization_manager/README.md`
- `base_ws/src/bringup/package.xml`
- `scripts/evaluation/fast_lio/eval_fast_lio_state_with_rviz.py`
  - 기존 기본값은 유지
  - startup transient 이후의 시점을 정렬 기준으로 고를 수 있는
    `--alignment-reference-sec` 추가

### 제거

- 없음

FAST-LIO, FAST-LIO MSF submodule과 MORAI UDP receiver는 수정하지 않았다.

## 8. 빌드

```bash
# GNSS+IMU localization 패키지와 bringup을 빌드
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
catkin build localization_manager bringup
source devel/setup.bash
```

## 9. 기존 raw bag으로 재녹화

대상 raw bag:

```text
rosbags/raw_sensor_w_cam/kcity_eval_v1_run01.bag
```

이 bag에는 융합 결과가 없으므로 raw GNSS/IMU를 재생하면서 새 provider의
중간 및 최종 토픽을 새 bag에 기록한다. 정확한 EKF timing을 위해 성능용
재녹화는 `--rate 1`로 수행한다. 4배속 재생은 smoke test에는 가능하지만
성능 비교용으로 사용하지 않는다.

### 터미널 1: ROS master

```bash
# ROS master 실행
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roscore
```

### 터미널 2: GNSS+IMU provider

```bash
# bag의 /clock을 사용하는 noise-OFF GNSS+IMU localization 실행
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch bringup localization_gnss_imu.launch \
  use_sim_time:=true \
  sensor_config:=/home/luv/AjouNice2026/configs/sensors/Sensor_4.json \
  measurement_noise_config:=/home/luv/AjouNice2026/configs/localization/gnss_imu_noise_sensor4_ideal.yaml
```

아직 bag을 재생하지 않았으면 `/use_sim_time set to true and no clock
published` warning은 정상이다.

### 터미널 3: 결과 녹화

```bash
# 입력·중간 measurement·EKF 출력·TF를 평가용 bag에 기록
cd ~/AjouNice2026
source base_ws/devel/setup.bash

GNSS_IMU_EVAL_BAG="$HOME/AjouNice2026/rosbags/eval/localization/gnss_imu_kcity_eval_v1_run01.bag" \
  bash scripts/rosbag/record/rosbag_for_gnss_imu_eval.sh
```

`Recording to`가 나타난 뒤 터미널 4를 시작한다.

### 터미널 4: raw bag 재생

```bash
# bag 경로를 --topics보다 먼저 쓰고 1배속으로 재생
cd ~/AjouNice2026
source base_ws/devel/setup.bash

rosbag play rosbags/raw_sensor_w_cam/kcity_eval_v1_run01.bag \
  --clock \
  --pause \
  --rate 1 \
  --topics \
    /Ego_topic \
    /sensor/gps \
    /sensor/imu
```

Space를 눌러 재생한다. 재생이 끝나면 터미널 3에서 `Ctrl-C`를 눌러
녹화를 종료한다.

## 10. MORAI 실시간 녹화

### 터미널 1: ROS master

```bash
# ROS master 실행
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roscore
```

### 터미널 2: MORAI UDP 수신

```bash
# Sensor_4 GNSS와 IMU를 ROS 토픽으로 수신
cd ~/AjouNice2026/base_ws
source devel/setup.bash

roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:=/home/luv/AjouNice2026/configs/sensors/Sensor_4.json \
  network_config:=/home/luv/AjouNice2026/configs/network/ego_network_1.json
```

### 터미널 3: GNSS+IMU provider

```bash
# 실시간 wall time을 사용하는 GNSS+IMU localization 실행
cd ~/AjouNice2026/base_ws
source devel/setup.bash

roslaunch bringup localization_gnss_imu.launch \
  use_sim_time:=false \
  sensor_config:=/home/luv/AjouNice2026/configs/sensors/Sensor_4.json \
  measurement_noise_config:=/home/luv/AjouNice2026/configs/localization/gnss_imu_noise_sensor4_ideal.yaml
```

이미 `sensing.launch`가 sensor static TF를 발행 중이면 중복을 막기 위해
`publish_sensor_tf:=false`도 전달한다.

### 터미널 4: 주기와 TF 확인

```bash
# 전처리와 최종 출력 주기 확인
rostopic hz --wall-time /localization/imu
rostopic hz --wall-time /localization/gnss/odom
rostopic hz --wall-time /localization/kinematic_state

# TF ownership 확인
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_link
```

### 터미널 5: 평가용 녹화

```bash
# 실시간 평가용 GNSS+IMU bag 기록
cd ~/AjouNice2026
source base_ws/devel/setup.bash
bash scripts/rosbag/record/rosbag_for_gnss_imu_eval.sh
```

## 11. 수치 평가

평가기는 이름에 `fast_lio`가 남아 있지만
`nav_msgs/Odometry` 형식의 `/localization/kinematic_state`를 평가하므로
GNSS+IMU provider에도 재사용한다.

K-City raw bag은 약 1.2초 동안 IMU yaw가 `62.586°`로 유지된 뒤 Ego
heading `61.45°` 부근으로 정상화되는 startup transient가 있다. 기존
평가기의 첫 표본 정렬은 이 `1.136°` 차이로 전체 268 m 궤적을 잘못
회전시켰다. 그래서 이 bag에만 2초 표본을 one-time alignment 기준으로
사용한다.

이 옵션은 초기 2초를 평가에서 삭제하지 않는다. 해당 구간의 yaw와
speed 오차는 그대로 계산하며, 단지 불안정한 첫 yaw가 전체 좌표 정렬을
정의하지 않게 한다.

```bash
# K-City GNSS+IMU 위치·속도·yaw-rate 평가
cd ~/AjouNice2026
source base_ws/devel/setup.bash

python3 scripts/evaluation/fast_lio/eval_fast_lio_state_with_rviz.py \
  rosbags/eval/localization/gnss_imu_kcity_eval_v1_run01.bag \
  --state-topic /localization/kinematic_state \
  --sensor-config configs/sensors/Sensor_4.json \
  --ego-velocity-unit kmph \
  --alignment-reference-sec 2.0 \
  --out rosbags/eval/localization/gnss_imu_kcity_eval_v1_run01.csv \
  --position-error-threshold 1.0 \
  --speed-error-threshold 0.5 \
  --yaw-rate-error-threshold 0.1
```

새로 녹화할 때는 MORAI/센서를 먼저 안정화한 뒤 녹화를 시작하고,
startup transient가 없으면 기본값인
`--alignment-reference-sec 0.0`을 사용한다. 수치를 좋게 만들기 위해
임의로 alignment 시간을 바꾸면 안 된다.

## 12. RViz 시각화

아래 절차는 GT path, GNSS+IMU path, threshold 초과 marker, TF, K-City
Lanelet map을 한 화면에 표시한다.

### 터미널 1: ROS master와 simulation time

```bash
# 녹화 bag의 timestamp를 RViz에서 사용
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roscore
```

다른 터미널에서:

```bash
rosparam set /use_sim_time true
```

### 터미널 2: K-City map

```bash
# Lanelet map 발행
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roslaunch lanelet_map_visualizer kcity_lanelet_map.launch
```

### 터미널 3: 평가 path와 marker 발행

```bash
# 평가 결과를 latched RViz 토픽으로 발행
cd ~/AjouNice2026
source base_ws/devel/setup.bash

python3 scripts/evaluation/fast_lio/eval_fast_lio_state_with_rviz.py \
  rosbags/eval/localization/gnss_imu_kcity_eval_v1_run01.bag \
  --state-topic /localization/kinematic_state \
  --sensor-config configs/sensors/Sensor_4.json \
  --ego-velocity-unit kmph \
  --alignment-reference-sec 2.0 \
  --out rosbags/eval/localization/gnss_imu_kcity_eval_v1_run01.csv \
  --position-error-threshold 1.0 \
  --speed-error-threshold 0.5 \
  --yaw-rate-error-threshold 0.1 \
  --publish-rviz
```

### 터미널 4: TF 시간 재생

```bash
# map→odom→base_link TF를 시간 순서대로 재생
cd ~/AjouNice2026
source base_ws/devel/setup.bash

rosbag play rosbags/eval/localization/gnss_imu_kcity_eval_v1_run01.bag \
  --clock \
  --pause \
  --topics \
    /tf \
    /tf_static
```

Space로 재생/일시정지한다.

### 터미널 5: RViz

```bash
# 준비된 GNSS+IMU 평가 레이아웃으로 RViz 실행
cd ~/AjouNice2026/base_ws
source devel/setup.bash

rviz -d "$(rospack find localization_manager)/rviz/gnss_imu_evaluation.rviz"
```

기본 Fixed Frame은 `morai_map`이다. Lanelet map은 기록된
`map → morai_map` static TF를 통해 같은 화면에 표시된다.

## 13. 노이즈 ON 후속 실험

MORAI에서 `Sensor_4_noise_v1.json`을 실제로 load하고, UDP와
localization 양쪽에 같은 센서 파일을 전달해야 한다.

```bash
# noise-v1 센서와 이에 대응하는 covariance profile로 실행
cd ~/AjouNice2026/base_ws
source devel/setup.bash

roslaunch bringup localization_gnss_imu.launch \
  use_sim_time:=false \
  sensor_config:=/home/luv/AjouNice2026/configs/sensors/Sensor_4_noise_v1.json \
  measurement_noise_config:=/home/luv/AjouNice2026/configs/localization/gnss_imu_noise_sensor4_noise_v1.yaml
```

노이즈 OFF와 ON은 같은 경로, 속도, 정지 시간으로 각각 여러 번 녹화해야
평균과 분산을 비교할 수 있다.

## 14. 현재 한계

- 이 provider는 GNSS position, IMU yaw/yaw-rate/acceleration만 사용한다.
- GNSS outage 동안에는 IMU 적분 오차가 빠르게 증가한다.
- 현재 속도는 GNSS 위치와 IMU acceleration으로 EKF가 간접 추정한다.
  별도 wheel speed 또는 GNSS velocity를 직접 융합하지 않아 정지·가감속
  속도 정확도가 위치보다 낮다.
- raw bag의 동일 timestamp IMU 중복은 adapter가 의도적으로 제거한다.
- YAML의 filter output 목표는 30 Hz지만 이번 bag replay에서 최종 출력은
  약 14.9 Hz였다. 입력 IMU/GNSS도 약 15 Hz였으나
  `robot_localization`은 설정상 입력 사이에도 predict output을 낼 수
  있으므로 단순히 입력 주기 때문이라고 단정하지 않는다. 실제 MORAI
  wall-time에서 주기를 다시 측정하고, 제어가 30 Hz state를 요구하면 ROS
  sim-time timer/CPU/TF scheduling을 추가 조사해야 한다.
- `Sensor_4_noise_v1` covariance는 아직 반복 실험으로 보정하지 않았다.
- 실대회 사용 결정 전에는 noise, GNSS outage, 장기 직선, 회전, 정지/출발
  시나리오를 모두 비교해야 한다.
