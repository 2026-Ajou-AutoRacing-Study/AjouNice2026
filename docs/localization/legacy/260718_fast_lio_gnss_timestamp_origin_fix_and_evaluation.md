# 260718 FAST-LIO GNSS timestamp 원점 초기화 수정 및 평가

## 목적

2026-07-17에 반복 GNSS correction의 측정 시각을 맞췄지만, 첫 GNSS 상대좌표
원점은 여전히 ROS callback 실행 순간의 최신 EKF 상태로 계산되고 있었다.
이 문서는 첫 GNSS 원점까지 측정 timestamp 기준으로 초기화하도록 수정한
내용과 실제 워크스페이스 평가 결과를 기록한다.

## 수정 전 문제

수정 전 첫 GNSS callback은 다음 순서로 동작했다.

```text
첫 GNSS callback 실행
-> callback 실행 순간의 kf.get_x() 조회
-> 해당 상태로 camera_gnss_origin 계산
```

```
# 예시
GNSS 측정 시각:       10.00초
GNSS callback 실행:   10.04초
kf.get_x()의 상태:    10.04초 차량 상태
# GNSS 메시지는 10.00초의 측정값이지만, ROS callback은 시스템 부하 때문에 10.04초에 실행될 수 있습니다.

10.00초 GNSS 위치
+
10.04초 FAST-LIO 차량 상태
→ GNSS 원점 계산
```

GNSS 메시지의 timestamp가 같더라도 callback이 실행되는 순서는 CPU 부하,
rosbag 기록 토픽 수, 디스크 I/O 등에 따라 달라질 수 있다. 실제로 같은 raw
bag을 사용한 두 실행에서 초기 원점이 약 3.42 cm 달랐고, 첫 GNSS update부터
`/localization/odom` 궤적이 달라졌다.

## 수정 방식

이제 첫 GNSS callback에서는 원점을 계산하지 않습니다.
```
첫 GNSS 도착
→ GNSS 값과 timestamp 10.00초를 버퍼에 저장
→ 원점 계산은 잠시 보류
```

FAST-LIO는 IMU를 처리하면서 차량 상태를 시간순으로 전파합니다.
```
9.98초 차량 상태
9.99초 차량 상태
10.00초 차량 상태  ← GNSS timestamp와 일치
10.01초 차량 상태
...
IMU propagation이 10.00초에 도달하면, 바로 그때의 상태를 별도로 저장합니다.
```
10.00초의 IMU 위치
10.00초의 차량 회전
10.00초의 GNSS 안테나 예측 위치
```
그다음 아래처럼 같은 시각끼리 원점을 정렬합니다.
```
GNSS 측정:              10.00초
FAST-LIO 안테나 상태:   10.00초
→ GNSS 원점 계산
```
ROS callback이 실제로 10.02초, 10.04초, 10.08초 중 언제 실행됐는지는 더 이상 중요하지 않습니다.

첫 GNSS 측정은 즉시 EKF 상태와 결합하지 않고 timestamp와 함께 버퍼링한다.
이후 IMU propagation이 해당 timestamp에 도달했을 때 다음 값을 저장한다.

- 해당 시각의 예측 GNSS 안테나 위치
- 해당 시각의 예측 body 회전
- 원본 GNSS 측정과 covariance

post-LiDAR GNSS 단계에서 다음과 같이 원점을 초기화한다.

```text
camera_gnss_origin
    = predicted_antenna_position(t_first_gnss)

camera_from_external_gnss_rotation
    = predicted_body_rotation(t_first_gnss)
    * inverse(external_initial_base_rotation)
```

첫 GNSS 샘플은 상대좌표 원점을 정의하는 용도로만 사용하고 위치 correction은
수행하지 않는다. 두 번째 GNSS부터 local displacement를 위 회전·이동으로
`camera_init` 좌표계에 변환하여 timestamp가 일치하는 예측 위치와 비교한다.

## 수정 파일

### 서브모듈

- `base_ws/src/localization/fast_lio_multi_sensor_fusion/src/IMU_Processing.hpp`
  - GNSS timestamp의 예측 안테나 위치와 함께 body 회전을 보관·반환하도록 수정
- `base_ws/src/localization/fast_lio_multi_sensor_fusion/src/laserMapping.cpp`
  - callback 시점 원점 초기화를 제거
  - 첫 GNSS local displacement를 0으로 버퍼링
  - timestamp가 일치하는 propagation 상태에서 원점과 회전 초기화
  - 첫 샘플에는 covariance floor와 위치 correction을 적용하지 않음
- `base_ws/src/localization/fast_lio_multi_sensor_fusion/README.md`
  - AjouNice GNSS timestamp 융합 방식 안내 추가
- `base_ws/src/localization/fast_lio_multi_sensor_fusion/AJOUNICE_CHANGES.md`
  - 재현성 문제, 구현 및 평가 결과 추가

### 메인 저장소

- `docs/localization/260718_fast_lio_gnss_timestamp_origin_fix_and_evaluation.md`
  - 현재 문서
- `260718_fast_lio_gnss_reproducibility_summary.md`
  - 팀원 공유용 원인·해결 요약

제거된 파일은 없다.

## 빌드 검증

```bash
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
catkin build fast_lio_msf localization_manager bringup
source devel/setup.bash
```

결과:

- 29개 관련 패키지 모두 빌드 성공
- `fast_lio_msf` 빌드 성공
- 기존 PCL `uint16_t` deprecation warning만 발생
- 신규 compile error 없음

## 평가 절차

입력 bag:

```text
rosbags/raw1_beta_drive.bag
```

입력 토픽:

```text
/Ego_topic
/sensor/gps
/sensor/imu
/sensor/lidar1
```

실행은 `use_sim_time:=true`, `rosbag play --clock --rate 0.5` 조건으로 수행했다.
평가 중에는 사용자 실행과 유사한 부하를 만들기 위해 raw 센서, point cloud,
TF, GNSS 디버그 토픽까지 모두 기록했다.

생성된 결과:

```text
rosbags/eval/260718_fast_lio_gnss_timestamp_origin_full.bag
rosbags/eval/260718_fast_lio_gnss_timestamp_origin_eval.csv
rosbags/eval/260718_fast_lio_gnss_timestamp_origin_eval.txt
```

평가 명령:

```bash
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
cd ~/AjouNice2026

python3 scripts/evaluation/localization/eval_localization_state_with_rviz.py \
  rosbags/eval/260718_fast_lio_gnss_timestamp_origin_full.bag \
  --state-topic /localization/kinematic_state \
  --out rosbags/eval/260718_fast_lio_gnss_timestamp_origin_eval.csv
```

## 첫 GNSS 원점 결과

```text
first GNSS timestamp: 107.973000000 s
camera GNSS origin:  [-1.493769849, -0.027259919, 1.036219187]
```

임시 검증본의 가벼운 녹화와 실제 코드의 전체 녹화에서 위 값이 동일했다.
두 실행에 공통으로 존재하는 1,946개 샘플도 다음과 같이 일치했다.

```text
timestamp 최대 차이: 0 s
위치 평균 차이:      0 m
위치 RMSE 차이:      0 m
위치 최대 차이:      0 m
```

## 최종 평가 결과

| 지표 | 결과 |
|---|---:|
| matched samples | 2,014 |
| duration | 103.780 s |
| Ego path length | 316.815 m |
| FAST-LIO path length | 333.556 m |
| XY mean error | 0.104 m |
| XY RMSE | 0.126 m |
| XY maximum error | 0.548 m |
| XY final error | 0.150 m |
| final error / path length | 0.047% |
| yaw mean absolute error | 1.080 deg |
| yaw maximum absolute error | 1.854 deg |

차량 좌표계 오차:

| 지표 | Bias | MAE | RMSE | Max abs |
|---|---:|---:|---:|---:|
| 종방향 | -0.035 m | 0.053 m | 0.074 m | 0.548 m |
| 횡방향 | -0.015 m | 0.079 m | 0.101 m | 0.389 m |

최종 시점 오차:

```text
종방향: -0.145 m
횡방향: +0.036 m
XY:      0.150 m
```

## 이전 결과와 비교

| 실행 | XY mean | XY RMSE | XY max | XY final |
|---|---:|---:|---:|---:|
| 첫 timestamp correction 사용자 실행 | 0.142 m | 0.159 m | 0.561 m | 0.152 m |
| 과거 경량 `/tmp` 실행 | 0.104 m | 0.130 m | 0.524 m | 0.169 m |
| 첫 원점 timestamp 수정 실제 코드 | 0.104 m | 0.126 m | 0.548 m | 0.150 m |

최종 실제 코드 결과는 과거 경량 `/tmp` 결과 수준을 재현했으며, 녹화 부하에
따라 첫 GNSS 원점과 궤적이 달라지던 현상은 이번 bag에서 제거됐다.

## 남은 문제

- FAST-LIO path length가 GT보다 16.740 m 길다.
- 정지 구간에서 비영 속도와 위치 jitter가 남아 있다.
- speed MAE는 0.654 m/s로 위치 정확도에 비해 크다.
- GNSS dropout, outlier, 재수신 및 noisy GNSS 조건은 아직 검증하지 않았다.
- 이번 수정은 임의의 과거 측정까지 상태를 rewind하는 완전한 OOSM 처리는 아니다.

따라서 첫 GNSS 원점 재현성은 개선됐지만, ZUPT 또는 별도의 velocity-state
보정은 후속 작업으로 다뤄야 한다.
