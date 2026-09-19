# 260717 legacy Ego bag을 이용한 FAST-LIO GNSS 평가

## 1. 목적

`morai_msgs`를 `beta_drive` 브랜치로 교체하면서
`morai_msgs/EgoVehicleStatus`의 정의와 MD5가 변경됐다.

```text
legacy raw1.bag: 861828353f677b0c61ddd2cdf47e398c
beta_drive:      a54480c8f4c41293a77cf7a0eb3925c9
```

ROS subscriber는 같은 타입 이름이어도 MD5가 다르면 연결되지 않는다. 따라서
기존 `raw1.bag`을 그대로 재생하면 현재
`morai_map_camera_init_broadcaster.py`가 `/Ego_topic`을 받지 못하고
`source: ego` 초기 정렬을 수행할 수 없다.

이 문서의 절차는 원본 bag은 보존하면서 `/Ego_topic`만 현재 메시지로
변환하고, 변환된 bag으로 FAST-LIO + GNSS를 다시 실행·녹화·평가·시각화한다.

## 2. 메시지 차이와 변환 범위

legacy 메시지에만 존재하고 `beta_drive`에서 제거된 필드는 다음과 같다.

```text
angular_velocity
distance_left_lane_boundary
distance_right_lane_boundary
cross_track_error
```

변환기는 현재 메시지에도 존재하는 position, velocity, acceleration,
heading, 차량 입력 및 tire 관련 필드를 복사한다. 위 네 필드는 변환된
메시지에서는 제외되지만 원본 bag에는 그대로 남는다.

GPS 메시지의 MD5는 기존 bag과 현재 패키지가 동일하므로 변환하지 않는다.
LiDAR와 IMU를 포함한 나머지 토픽도 bag timestamp를 유지해 그대로 복사한다.
`/Object_topic` 같은 다른 legacy MORAI 메시지의 schema까지 변환하는 범용
도구는 아니므로 GNSS 재생 시 필요한 토픽만 선택해서 재생한다.

## 3. 변경 파일

### 추가

- `scripts/rosbag/repair/convert_legacy_ego_vehicle_status.py`
  - legacy Ego 메시지를 현재 `beta_drive` 메시지로 재작성한다.
  - 원본과 같은 출력 경로 또는 기존 출력 파일을 허용하지 않는다.
  - 임시 `.active` bag 작성이 성공한 경우에만 최종 출력명으로 교체한다.

### 수정

- `scripts/evaluation/localization/eval_localization_state_with_rviz.py`
  - 제거된 `EgoVehicleStatus.angular_velocity`를 직접 읽지 않는다.
  - old/new 메시지 모두 heading 시계열의 local regression으로 GT yaw rate를
    계산해 같은 기준으로 비교한다.

### 삭제

- 없음

기존 `260716_fast_lio_gnss_fusion.md`와 localization runtime
launch/config는 수정하지 않았다.

## 4. 사전 빌드

```bash
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
catkin build fast_lio_msf localization_manager bringup
source devel/setup.bash
```

## 5. legacy raw bag 변환

이 작업은 bag마다 한 번만 수행한다. `raw1.bag`과 비슷한 크기의 출력이
생기므로 약 2 GB 이상의 추가 여유 공간을 확보한다.

```bash
cd ~/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
cd ~/AjouNice2026

python3 scripts/rosbag/repair/convert_legacy_ego_vehicle_status.py \
  rosbags/raw1.bag \
  rosbags/raw1_beta_drive.bag
```

출력 확인:

```bash
rosbag info rosbags/raw1_beta_drive.bag
```

다음 MD5가 보여야 한다.

```text
morai_msgs/EgoVehicleStatus [a54480c8f4c41293a77cf7a0eb3925c9]
```

원본과 변환 bag의 주요 토픽 메시지 수도 함께 비교한다.

```bash
rosbag info rosbags/raw1.bag
rosbag info rosbags/raw1_beta_drive.bag
```

## 6. 변환 bag으로 GNSS 결과 재녹화

모든 터미널은 같은 ROS master와 현재 workspace 환경을 사용해야 한다.
`localization_fast_lio.launch`, `localization_morai_gt.launch` 등 다른 측위
provider는 동시에 실행하지 않는다.

### 터미널 1: ROS master

```bash
cd ~/AjouNice2026/base_ws
source devel/setup.bash
roscore
```

### 터미널 2: FAST-LIO + GNSS

bag의 `/clock`을 사용하므로 simulated time을 활성화한다. 기본
`providers/fast_lio_gnss/initialization/source: ego` 설정을 그대로 사용한다.

```bash
cd ~/AjouNice2026/base_ws
source devel/setup.bash

roslaunch bringup localization_fast_lio_gnss.launch \
  use_sim_time:=true
```

### 터미널 3: 평가용 결과 녹화

```bash
cd ~/AjouNice2026
source base_ws/devel/setup.bash

scripts/rosbag/record/rosbag_for_fast_lio_gnss_eval.sh
```

`Recording to ...`가 표시된 후 다음 터미널의 bag을 재생한다.

### 터미널 4: 변환 bag 재생

```bash
cd ~/AjouNice2026
source base_ws/devel/setup.bash

rosbag play rosbags/raw1_beta_drive.bag \
  --clock \
  --rate 0.5 \
  --pause \
  --topics \
    /Ego_topic \
    /sensor/gps \
    /sensor/imu \
    /sensor/lidar1
```

터미널 2와 3이 준비된 것을 확인하고 space를 눌러 재생한다. 재생이 끝난
뒤 녹화 터미널에서 `Ctrl-C`로 bag을 정상 종료한다.

## 7. 실행 중 확인

```bash
rostopic hz --wall-time /Ego_topic
rostopic hz --wall-time /localization/gnss/fix
rostopic hz --wall-time /localization/odom
rostopic hz --wall-time /localization/kinematic_state
rosrun tf tf_echo map base_link
```

launch 로그에서는 초기 정렬 완료 메시지와 GNSS update 적용 횟수 증가를
확인한다. 초기 IMU 수렴 중 제한적으로 나타나는 `No point, skip this scan!`은
정상일 수 있다.

## 8. pose + twist 통합 평가

녹화된 실제 파일명으로 `MMDD_HHMM`을 교체한다. `--out`으로 지정한 CSV와
같은 이름의 TXT가 함께 생성된다.

```bash
cd ~/AjouNice2026/base_ws
source devel/setup.bash
cd ~/AjouNice2026

python3 scripts/evaluation/localization/eval_localization_state_with_rviz.py \
  rosbags/eval/fast_lio_gnss_full_0717_2004.bag \
  --state-topic /localization/kinematic_state \
  --out rosbags/eval/fast_lio_gnss_eval_2.csv \
  --position-error-threshold 1.0 \
  --speed-error-threshold 0.5 \
  --yaw-rate-error-threshold 0.1
```

GT yaw rate는 old/new Ego 메시지 모두 heading local regression으로 계산한다.
따라서 legacy 메시지의 `angular_velocity` 유무가 평가 기준을 바꾸지 않는다.

## 9. RViz 시각화

### 터미널 1: 평가 Path와 Marker 발행 유지

```bash
cd ~/AjouNice2026/base_ws
source devel/setup.bash
cd ~/AjouNice2026

python3 scripts/evaluation/localization/eval_localization_state_with_rviz.py \
  rosbags/eval/fast_lio_gnss_full_0717_2004.bag \
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
rosbag play rosbags/eval/fast_lio_gnss_full_0717_2004.bag \
  --clock \
  --pause \
  --topics /cloud_registered /tf /tf_static
```

space로 재생한다. `/cloud_registered`는 latched topic이 아니므로 PointCloud2를
보려면 bag 재생이 필요하다.

### 터미널 3: RViz

```bash
cd ~/AjouNice2026/base_ws
source devel/setup.bash
rviz
```

RViz 설정:

```text
Fixed Frame: map

Path:        /localization_state_eval/ego_path
Path:        /localization_state_eval/state_aligned_path
MarkerArray: /localization_state_eval/error_markers
PointCloud2: /cloud_registered
```

Path와 Marker는 latched publisher이므로 평가 프로세스를 유지한 상태에서
RViz를 나중에 열어도 보인다. 전체 경로가 수백 m 크기이므로 화면에 보이지
않으면 먼저 충분히 줌아웃한다.

## 10. 실시간 MORAI 데이터

현재 MORAI UDP interface가 발행하는 Ego 메시지는 이미 `beta_drive` schema를
사용하므로 실시간 주행을 새로 녹화할 때는 변환기를 사용하지 않는다. 이
문서의 변환 단계는 legacy MD5로 녹화된 기존 bag에만 적용한다.
