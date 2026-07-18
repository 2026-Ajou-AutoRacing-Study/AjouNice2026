# 260716 FAST-LIO + GNSS global correction

## 1. 결론과 현재 상태

`fast_lio_multi_sensor_fusion` 포크를 MORAI 파이프라인에 연결해 LiDAR,
IMU, GPS를 사용하고 wheel 입력은 사용하지 않는 실험용 provider를 추가했다.

`raw1.bag` 전체 구간에서 위치 오차는 다음과 같았다.

| 항목 | FAST-LIO 단독 기존 결과 | FAST-LIO + GNSS 현재 결과 |
|---|---:|---:|
| 주행거리 | 약 316.7 m | 316.815 m |
| XY 평균 오차 | 4.388 m | 0.194 m |
| XY RMSE | 4.957 m | 0.253 m |
| XY 최대 오차 | 7.568 m | 0.722 m |
| XY 최종 오차 | 7.483 m | 0.323 m |
| 최종오차/주행거리 | 2.363% | 0.102% |
| 1 m 초과 위치 구간 | 장시간 존재 | 0개 |

따라서 GPS에 의한 global position correction 자체는 작동한다. 다만 같은
실험에서 속력 MAE가 0.747 m/s, yaw 평균 절대오차가 1.083 deg로 나왔다.
현재 provider는 **위치 보정 시제품**이며, planning의 기본 provider로 바로
교체하기 전 twist와 yaw를 추가 개선해야 한다.

## 2. 데이터 흐름과 TF

```text
/sensor/gps (morai_msgs/GPSMessage)
  -> morai_gps_to_navsatfix.py
  -> /localization/gnss/fix (sensor_msgs/NavSatFix)

/sensor/lidar1 + /sensor/imu + /localization/gnss/fix
  + /localization/initialpose
  -> fast_lio_msf
  -> /localization/odom (camera_init -> body)
  -> camera_init -> body TF

/localization/odom
  -> fast_lio_kinematic_state_publisher.py
  -> /localization/kinematic_state (map -> base_link pose/twist)
```

최종 TF tree는 다음과 같다.

```text
map -> morai_map -> camera_init -> body -> base_link
```

- `map -> morai_map`: 기존 `tf_pub/map_tf.launch`의 static TF
- `morai_map -> camera_init`: 시작 pose로 한 번 정렬한 뒤 같은 값을 동적
  timestamp로 발행
- `camera_init -> body`: FAST-LIO-MSF
- `body -> base_link`: 현재 Sensor_1/2 IMU 위치를 사용한 하드코딩 static TF

다른 localization provider와 TF·토픽 소유권이 겹치므로
`localization_fast_lio.launch`, `localization_morai_gt.launch`,
`maptf_morai.launch`를 동시에 실행하면 안 된다.

## 3. 시작 pose

설정은 `configs/localization/localization_manager.yaml`의 다음 항목이다.

```yaml
providers:
  fast_lio_gnss:
    initialization:
      source: ego  # ego | manual
```

### `ego`

개발 중에는 처음 동기화된 `/Ego_topic`과 `/localization/odom`으로
`morai_map -> camera_init`을 계산한다. 같은 base pose를 latched
`/localization/initialpose`로 MSF에 전달한다. Ego는 시작 정렬에 한 번만
사용하며 이후 위치 보정에는 `/sensor/gps`를 사용한다.

### `manual`

대회에서 제공되는 시작 pose를 아래 값에 넣어 사용할 수 있다.

```yaml
manual:
  frame_id: morai_map
  position: [0.0, 0.0, 0.0]
  rpy_degrees: [0.0, 0.0, 0.0]
```

manual 경로는 구현했지만 현재 bag으로는 `ego` 경로만 검증했다.

## 4. GNSS 처리에서 수정한 핵심 이유

### 4.1 MORAI GPS adapter

MORAI는 `morai_msgs/GPSMessage`를 발행하지만 MSF는
`sensor_msgs/NavSatFix`를 요구한다. launch 파일만으로 메시지 타입과
covariance를 계산해 바꿀 수 없으므로 Python adapter를 추가했다.

adapter는 LLA 범위, finite 값, timestamp 순서와 중복을 확인하고 임시
공분산을 채운다. 현재 기본값은 수평 표준편차 0.5 m, 수직 1.0 m이며 대회
센서 noise 범위를 받은 뒤 반드시 다시 정해야 한다.

### 4.2 UTM 사용

원본 코드는 true-north 기반 LocalCartesian ENU를 사용했다. MORAI map과
기존 `gps_tf_broadcaster`는 UTM grid 축을 사용한다. 기록 위치에서는 두
축의 meridian convergence 때문에 약 315 m 주행 시 횡방향으로 약 8 m가
달라졌다. MORAI provider에서는 UTM easting/northing 차를 사용하도록 했다.

### 4.3 timestamp 누락 수정

raw bag의 GPS와 IMU timestamp가 정확히 같은 샘플이 많았다. 원본의 엄격한
부등식 처리에서는 이 GPS가 대부분 버려졌다. 동일 timestamp를 포함하도록
IMU 구간 분할을 수정했고, 외부 정렬 모드에서는 LiDAR update 뒤 최신 GNSS를
한 번 적용한다. 전체 bag에서 약 1,950회의 GNSS update가 확인됐다.

### 4.4 position-only correction

단일 GPS 안테나는 위치를 직접 측정하지만 heading을 직접 측정하지 않는다.
원본 GNSS update를 강하게 적용하면 자세와 속도까지 공분산을 통해 변해 yaw와
twist가 악화됐다. 현재 외부 정렬 모드는 다음 residual을 사용해 위치만
보정한다.

```text
predicted_gnss = imu_position + R_camera_body * lever_arm_imu_to_gnss
innovation     = measured_gnss - predicted_gnss
```

위치 공분산과 GNSS 공분산으로 3D Kalman gain을 계산해 `state.pos`만 갱신한다.
GNSS update는 LiDAR update 이후, 보정된 scan을 incremental map에 넣기 전에
수행한다.

### 4.5 covariance

FAST-LIO 위치 공분산이 지나치게 작아지면 GNSS가 있어도 gain이 거의 0이
된다. 현재 adapter의 수평 분산 0.25 m²와 동일하게 위치 공분산 하한을
0.25 m²로 설정했다. 이것은 임시 튜닝값이며 실제 noise·outlier 실험 후
바꿔야 한다.

## 5. 변경 파일 목록

### 상위 repository에서 추가

- `base_ws/src/bringup/launch/fast_lio_msf_morai.launch`
  - MORAI용 FAST-LIO-MSF 알고리즘 실행
- `base_ws/src/localization/localization_manager/launch/fast_lio_gnss.launch`
  - GPS adapter, 시작 정렬, body static TF, kinematic publisher 실행
- `base_ws/src/localization/localization_manager/scripts/morai_gps_to_navsatfix.py`
  - MORAI GPS를 표준 NavSatFix로 변환
- `configs/localization/fast_lio_msf_morai_velodyne.yaml`
  - LiDAR/IMU/GNSS topic, extrinsic, projection, covariance 설정
- `scripts/rosbag/record/rosbag_for_fast_lio_gnss_eval.sh`
  - 입력·결과·TF·RViz용 점군을 함께 기록

### 상위 repository에서 수정

- `base_ws/src/bringup/launch/localization_fast_lio_gnss.launch`
  - 임시 placeholder를 최상위 provider launch로 교체
- `base_ws/src/localization/localization_manager/CMakeLists.txt`
  - 새 Python adapter 설치 등록
- `base_ws/src/localization/localization_manager/package.xml`
  - `sensor_msgs` 의존성 추가
- `base_ws/src/localization/localization_manager/README.md`
  - GNSS provider 계약과 제한사항 추가
- `base_ws/src/localization/localization_manager/launch/fast_lio.launch`
  - 기존 provider 이름을 명시해 설정 분리
- `base_ws/src/localization/localization_manager/scripts/morai_map_camera_init_broadcaster.py`
  - `fast_lio`/`fast_lio_gnss`, `ego`/`manual`, initial pose 발행 지원
- `configs/localization/localization_manager.yaml`
  - GPS topic과 `fast_lio_gnss` provider 설정 추가

### submodule에서 수정

- `base_ws/src/localization/fast_lio_multi_sensor_fusion/src/GNSS_Processing.hpp`
- `base_ws/src/localization/fast_lio_multi_sensor_fusion/src/IMU_Processing.hpp`
- `base_ws/src/localization/fast_lio_multi_sensor_fusion/src/laserMapping.cpp`

submodule 상세 변경은
`base_ws/src/localization/fast_lio_multi_sensor_fusion/AJOUNICE_CHANGES.md`에
별도로 기록했다.

### 제거

- 삭제한 파일은 없다.
- `localization_fast_lio_gnss.launch` 파일 자체는 삭제하지 않고 placeholder
  내용을 실제 launch 구성으로 교체했다.

## 6. 빌드

```bash
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
catkin build fast_lio_msf localization_manager bringup
source devel/setup.bash
```

## 7. 기존 `raw1.bag`으로 재녹화

모든 터미널에서 같은 ROS master를 사용해야 한다. 기존 ROS master가 없다면
터미널 1에서 실행한다.

### 터미널 1: ROS master

```bash
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roscore
```

### 터미널 2: GNSS localization

bag의 `/clock`을 사용할 것이므로 `use_sim_time:=true`가 맞다.

```bash
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roslaunch bringup localization_fast_lio_gnss.launch use_sim_time:=true
```

### 터미널 3: 결과 녹화

```bash
cd ~/AjouNice2026
source base_ws/devel/setup.bash
scripts/rosbag/record/rosbag_for_fast_lio_gnss_eval.sh
```

`Recording to ...`가 나온 뒤 다음 터미널을 실행한다. 종료는 `Ctrl-C`로 해도
되며 스크립트가 recorder에 SIGINT를 전달해 bag을 정상 마감한다.

### 터미널 4: raw bag 재생

```bash
cd ~/AjouNice2026
source base_ws/devel/setup.bash
rosbag play rosbags/raw1.bag \
  --clock \
  --rate 0.5 \
  --pause \
  --topics \
    /Ego_topic \
    /sensor/gps \
    /sensor/imu \
    /sensor/lidar1
```

터미널 2와 3이 준비된 것을 확인한 뒤 space를 눌러 재생한다.
`rosbag play`에는 `--exclude` 옵션이 없으며, `--clock`이 재생 시간에 맞는
`/clock`을 생성하므로 `/clock`을 `--topics`에 따로 넣지 않는다.

## 8. 평가

녹화된 파일명을 확인한 뒤 다음처럼 실행한다. `--out`의 `.csv`와 같은
이름의 `.txt`가 함께 생성된다.

```bash
cd ~/AjouNice2026/base_ws
source devel/setup.bash
cd ~/AjouNice2026

python3 scripts/evaluation/fast_lio/eval_fast_lio_state_with_rviz.py \
  rosbags/eval/fast_lio_gnss_full_MMDD_HHMM.bag \
  --state-topic /localization/kinematic_state \
  --out rosbags/eval/fast_lio_gnss_eval_1.csv \
  --position-error-threshold 1.0 \
  --speed-error-threshold 0.5 \
  --yaw-rate-error-threshold 0.1
```

현재 `raw1.bag`에는 Ego GT가 있으므로 pose와 twist를 함께 평가할 수 있다.
대회 실전에는 Ego GT가 없으므로 이 평가는 개발용이다.

## 9. RViz 시각화

### 터미널 1: 평가 결과 publisher 유지

```bash
cd ~/AjouNice2026/base_ws
source devel/setup.bash
cd ~/AjouNice2026

python3 scripts/evaluation/fast_lio/eval_fast_lio_state_with_rviz.py \
  rosbags/eval/fast_lio_gnss_full_MMDD_HHMM.bag \
  --state-topic /localization/kinematic_state \
  --position-error-threshold 1.0 \
  --speed-error-threshold 0.5 \
  --yaw-rate-error-threshold 0.1 \
  --publish-rviz
```

### 터미널 2: 점군과 TF 재생

```bash
cd ~/AjouNice2026/base_ws
source devel/setup.bash
cd ~/AjouNice2026

rosparam set /use_sim_time true
rosbag play rosbags/eval/fast_lio_gnss_full_MMDD_HHMM.bag \
  --clock \
  --pause \
  --topics /cloud_registered /tf /tf_static
```

space로 재생한다. `/cloud_registered`는 latched topic이 아니므로 점군을
보려면 bag을 실제로 재생해야 한다.

### 터미널 3: RViz

```bash
cd ~/AjouNice2026/base_ws
source devel/setup.bash
rviz
```

RViz 설정:

- Fixed Frame: `map`
- Path: `/fast_lio_state_eval/ego_path` (GT)
- Path: `/fast_lio_state_eval/lio_aligned_path` (GNSS-corrected result)
- MarkerArray: `/fast_lio_state_eval/error_markers`
- PointCloud2: `/cloud_registered`

경로는 latched publisher라 평가 프로세스를 켜 둔 뒤 RViz를 열어도 보인다.
안 보이면 먼저 수백 m 크기의 전체 경로가 들어오도록 줌아웃한다.

## 10. 정상 로그와 확인 항목

초기 IMU 수렴 중 다음 로그는 제한적으로 정상이다.

```text
No point, skip this scan!
IMU Initial Done
```

초기 TF가 연결되는 한두 frame 동안 extrapolation/tree warning이 한 번 나올
수도 있다. 이후에는 다음을 확인한다.

```bash
rostopic hz --wall-time /localization/odom
rostopic hz --wall-time /localization/kinematic_state
rostopic hz --wall-time /localization/gnss/fix
rosrun tf tf_echo map base_link
```

launch 로그의 `Post-LiDAR GNSS updates applied` 숫자가 증가해야 실제 GNSS
update가 적용되는 것이다.

## 11. 다음 개발 순서

1. MORAI GPS noise를 여러 단계로 넣고 covariance를 보정한다.
2. GNSS outlier, 끊김, 재연결 시나리오를 추가한다.
3. FAST-LIO 단독과 MSF의 yaw·twist 차이를 같은 bag으로 직접 회귀 평가한다.
4. 정지 구간의 약 0.9 m/s 속력 오차를 우선 해결한다.
5. 위치 correction이 incremental LiDAR map에 주는 영향을 장거리·loop에서
   확인한다.
6. 검증이 끝날 때까지 planning 기본 provider는 기존 FAST-LIO 또는 MORAI GT로
   유지하고, GNSS provider는 선택 실행한다.

