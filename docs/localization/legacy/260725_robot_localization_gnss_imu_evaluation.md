# 260725 GNSS + IMU baseline 평가 결과

## 평가 조건

| 항목 | 값 |
|---|---|
| 원본 | `rosbags/raw_sensor_w_cam/kcity_eval_v1_run01.bag` |
| 재녹화 | `rosbags/eval/localization/260725_gnss_imu_kcity_eval_v1_run01.bag` |
| 센서 설정 | `Sensor_4.json`, MORAI noise OFF |
| fusion | ROS Noetic `robot_localization`, local/global EKF |
| GT | `/Ego_topic` |
| 평가 state | `/localization/kinematic_state` |
| Ego 속도 단위 | legacy km/h를 평가 시 m/s로 변환 |
| 재생 속도 | 1× |
| 평가 구간 | 60.595 s, 268.073 m |
| 정렬 기준 | 첫 표본으로부터 2.004 s |

## startup 정렬 검증

이 bag의 첫 IMU yaw는 약 1.2초 동안 `62.586°`에 머무르는 반면 같은
시점의 Ego heading은 `61.450°`였다. 이후에는 두 값의 차이가 거의 0으로
수렴했다.

GPS와 Ego의 전체 변위 방향 차이는 `-0.0048°`에 불과했다. 따라서 첫
표본 yaw 차이 `1.136°`를 전체 trajectory 정렬에 사용하면 약 268 m
주행에서 수 m의 가짜 횡방향 오차가 생긴다.

평가기는 `--alignment-reference-sec 2.0`을 사용했다. 초기 transient
표본은 삭제하지 않았으며 yaw와 speed 지표에 그대로 포함했다.

## 토픽과 TF 검증

| 토픽 | messages | 고유 timestamp 기준 주기 |
|---|---:|---:|
| `/localization/imu` | 922 | 15.183 Hz |
| `/localization/gnss/odom` | 893 | 14.705 Hz |
| `/localization/odometry/local` | 902 | 14.853 Hz |
| `/localization/odom` | 906 | 14.919 Hz |
| `/localization/kinematic_state` | 906 | 14.919 Hz |

확인한 TF edge:

```text
map → morai_map
map → odom
odom → base_link
base_link → imu
base_link → gps
```

모든 검사 대상 pose/twist/covariance 값은 finite였다.

## 위치 결과

| 지표 | 결과 |
|---|---:|
| GT path length | 268.073 m |
| 추정 path length | 270.597 m |
| path length error | +2.524 m |
| XY mean | 0.172 m |
| XY RMSE | 0.195 m |
| XY max | 0.413 m |
| XY final | 0.020 m |
| final drift / GT distance | 0.007% |
| yaw mean absolute error | 0.191° |
| yaw max absolute error | 1.570° |

차량 좌표계 위치 오차:

| 축 | bias | MAE | RMSE | max abs |
|---|---:|---:|---:|---:|
| 종방향 | -0.147 m | 0.153 m | 0.181 m | 0.413 m |
| 횡방향 | -0.032 m | 0.057 m | 0.073 m | 0.220 m |

제곱오차 기여도는 종방향 86.2%, 횡방향 13.8%였다. 위치 1.0 m threshold
초과 구간은 없었다.

고정 `map → morai_map` TF를 직접 적용해 초기 재정렬 없이 별도로 확인한
결과도 mean 0.146 m, RMSE 0.167 m, max 0.386 m, final 0.044 m였다.
따라서 2초 alignment로 결과를 인위적으로 만든 것이 아니라 고정 map
정합과도 일치한다.

## 속도와 yaw-rate 결과

| 지표 | bias | MAE | RMSE | max abs |
|---|---:|---:|---:|---:|
| linear.x | -0.519 m/s | 0.590 m/s | 0.860 m/s | 4.570 m/s |
| linear.y | +0.042 m/s | 0.498 m/s | 0.842 m/s | 3.687 m/s |
| speed | -0.230 m/s | 0.562 m/s | 0.831 m/s | 4.886 m/s |
| angular.z | +0.0027 rad/s | 0.0184 rad/s | 0.0359 rad/s | 0.2073 rad/s |

위치는 noise-OFF GNSS가 계속 보정하므로 매우 양호했다. 반면 속도는 wheel
speed나 GNSS velocity를 직접 입력하지 않고 IMU acceleration과 GNSS
position의 상태 전이를 통해 간접 추정하므로 성능이 낮다. 특히 초기
4.886 m/s spike와 정지 구간의 잔류 속도가 있다.

따라서 현재 결과는 다음처럼 해석해야 한다.

- GNSS가 정상인 noise-OFF 환경의 위치 baseline으로는 유효하다.
- 현재 형태 그대로 제어용 고품질 속도원으로 사용하는 것은 권장하지 않는다.
- 다음 우선순위는 wheel speed 또는 안정적으로 미분한 GNSS velocity를
  별도 measurement로 융합하고 정지 상태 속도 성능을 재평가하는 것이다.
- 그 다음 noise-ON 및 GNSS outage에서 FAST-LIO+GNSS와 비교해야 한다.

## FAST-LIO+GNSS와의 관계

이 결과만으로 GNSS+IMU가 FAST-LIO+GNSS보다 전반적으로 우수하다고
결론내리면 안 된다.

- 현재 bag은 GNSS noise가 꺼져 있어 위치가 사실상 GT에 가깝다.
- GNSS+IMU는 LiDAR가 없어 계산량이 작지만 GNSS outage와 noise에 취약하다.
- FAST-LIO+GNSS는 환경 geometry가 충분하면 GNSS가 흔들리거나 끊겨도
  LiDAR/IMU 상대운동으로 버틸 수 있다.
- 현재 GNSS+IMU의 속도 결과는 FAST-LIO 계열보다 개선이 필요하다.

따라서 GNSS+IMU는 현재 “완전 대체”가 아니라 비교 baseline 및 fallback
후보이다.

## 생성 파일

- `rosbags/eval/localization/260725_gnss_imu_kcity_eval_v1_run01.bag`
- `rosbags/eval/localization/260725_gnss_imu_kcity_eval_v1_run01.csv`
- `rosbags/eval/localization/260725_gnss_imu_kcity_eval_v1_run01.txt`

상세 실행 명령은
`260725_robot_localization_gnss_imu_implementation.md`에
정리했다.

## 260728 초기 pose 정책 추가 검증

FAST-LIO+GNSS와 동일하게 첫 `/Ego_topic`을 개발용 시작 pose로 사용하고,
대회 제공 pose로 교체할 수 있는 `ego | manual` 초기화 경로를 GNSS+IMU
global EKF에도 연결했다. 출력 readiness gate는 적용하지 않았다.

동일 raw bag과 동일 covariance 설정으로 변경 전후 startup을 비교했다.

| bag | 지표 | 변경 전 | 변경 후 |
|---|---|---:|---:|
| K-City | initial pose 이후 첫 matched state speed | 4.886 m/s | 0.000002 m/s |
| K-City | 첫 2초 최대 speed error | 4.886 m/s | 0.00024 m/s |
| Obstacle2 | initial pose 이후 첫 matched state speed | 27.915 m/s | 0.000001 m/s |
| Obstacle2 | 첫 2초 최대 speed error | 27.831 m/s | 0.258 m/s |
| Obstacle2 | 첫 2초 최대 XY error | 0.530 m | 0.187 m |

K-City의 raw IMU orientation이 약 1.48초 동안 Ego heading과 `1.136°`
다른 현상은 별개의 센서 입력 문제이므로 남아 있다. 초기 pose는 첫 GNSS
절대위치 보정에 따른 global EKF 속도 spike를 해결하지만, 이후 들어오는
잘못된 IMU orientation 자체를 수정하지는 않는다.

K-City에서는 첫 Ego가 첫 센서보다 약 32 ms 늦어 initial pose 발행 전에
`/localization/kinematic_state` 2개가 이미 발행됐으며, 이 구간 최대
속도는 `0.996 m/s`였다. initial pose 이후 첫 state 속도는
`0.000002 m/s`였다. 이번 변경은 요청대로 출력 gate를 포함하지 않으므로
initial pose 이전 메시지를 차단하지 않는다. 대회의 manual 시작 pose가
센서 fusion 전에 제공되면 이 선행 출력 구간도 피할 수 있고, 그렇지 않으면
별도 readiness gate가 필요하다.

검증 출력은 저장소를 변경하지 않도록 `/tmp`에만 생성했다.

- `/tmp/gnss_imu_initial_pose_kcity.bag`
- `/tmp/gnss_imu_initial_pose_kcity_ref2.txt`
- `/tmp/gnss_imu_initial_pose_obstacle2.bag`
- `/tmp/gnss_imu_initial_pose_obstacle2_ref2.txt`
