# Controller Tuning Workflow

이 문서는 `~/catkin_ws`에서 제어기 튜닝을 처음 하는 사람이 따라갈 수 있도록 정리한 실행 절차이다.

핵심 원칙:

```text
튜닝 이름 = 제어기 파라미터 YAML 이름 = 주행 결과 CSV 이름
```

예를 들어 튜닝 이름이 `baseline`이면 다음 두 파일이 한 세트이다.

```text
configs/controller_tuning/baseline.yaml
validation_runs/baseline.csv
```

`baseline.yaml`은 제어기 파라미터를 저장하고, `baseline.csv`는 그 파라미터로 실제 주행한 궤적과 error 결과를 저장한다.

## Directory Structure

기준 workspace:

```text
~/catkin_ws
```

주요 파일 구조:

```text
~/catkin_ws/src/ajou_control
├── configs/
│   └── controller_tuning/
│       └── baseline.yaml
├── launch/
│   ├── mgeo_path_stack.launch
│   └── view_validation_runs.launch
├── scripts/
│   ├── run_mgeo_controller_tuning.sh
│   ├── pure_pursuit_pi_controller.py
│   ├── tracking_validator.py
│   └── validation_run_viewer.py
├── validation_runs/
│   └── baseline.csv
└── rviz/
    └── mgeo_alignment.rviz
```

초기 배포 상태에서는 저장 주행 데이터가 들어있지 않다.

```text
~/catkin_ws/src/ajou_control/validation_runs/
~/catkin_ws/src/ajou_trajectory_recorder/data/
```

위 폴더들은 비어 있는 것이 정상이다. 실제 주행/녹화를 수행하면 CSV가 생성된다.

## Tuning Profile YAML

제어기 파라미터는 YAML 파일로 저장한다.

기본 파일:

```text
~/catkin_ws/src/ajou_control/configs/controller_tuning/baseline.yaml
```

현재 baseline 예시:

```yaml
route_length_m: 120.0
target_speed_mps: 2.0
velocity_scale: 0.27778

steering_sign: -1.0
min_lookahead: 7.0
max_lookahead: 16.0
lookahead_gain: 1.0
max_steer_rad: 0.35
max_steer_rate: 0.8

enable_rviz: true
```

중요한 값:

- `steering_sign: -1.0`: MORAI 조향 방향 반대 보정
- `velocity_scale: 0.27778`: km/h를 m/s로 변환
- `min_lookahead`, `max_lookahead`, `lookahead_gain`: Pure Pursuit lookahead 튜닝값
- `max_steer_rad`: 조향각 제한
- `max_steer_rate`: 조향각 변화율 제한
- `target_speed_mps`: 목표 속도

## Create New Tuning Case

새 튜닝 케이스를 만들 때는 baseline을 복사해서 시작한다.

예시: `max_steer_rate_08` 케이스 생성

```bash
cp ~/catkin_ws/src/ajou_control/configs/controller_tuning/baseline.yaml \
   ~/catkin_ws/src/ajou_control/configs/controller_tuning/max_steer_rate_08.yaml
```

그 다음 YAML을 열어서 필요한 값만 바꾼다.

```bash
code ~/catkin_ws/src/ajou_control/configs/controller_tuning/max_steer_rate_08.yaml
```

예를 들어:

```yaml
max_steer_rate: 0.8
```

튜닝할 때는 한 번에 하나의 파라미터만 바꾸는 것이 좋다.
그래야 주행 결과가 무엇 때문에 달라졌는지 판단할 수 있다.

## Run Controller With Profile

빌드와 환경 설정:

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

baseline 실행:

```bash
~/catkin_ws/src/ajou_control/scripts/run_mgeo_controller_tuning.sh baseline
```

이 명령은 내부적으로 다음을 수행한다.

```text
configs/controller_tuning/baseline.yaml 읽기
-> mgeo_path_stack.launch 실행
-> controller 실행
-> validator 실행
-> validation_runs/baseline.csv 저장
-> RViz 실행
```

`baseline.csv`가 이미 있으면 새 주행 결과로 덮어쓴다.
즉 같은 튜닝 이름은 최신 1회 결과만 유지한다.

다른 튜닝 케이스 실행:

```bash
~/catkin_ws/src/ajou_control/scripts/run_mgeo_controller_tuning.sh max_steer_rate_08
```

결과:

```text
~/catkin_ws/src/ajou_control/validation_runs/max_steer_rate_08.csv
```

## Compare Runs In RViz

특정 run 하나만 보기:

```bash
source ~/catkin_ws/devel/setup.bash

roslaunch ajou_control view_validation_runs.launch \
  input_csv:=$HOME/catkin_ws/src/ajou_control/validation_runs/baseline.csv \
  history_glob:= \
  enable_link_overlay:=true \
  enable_rviz:=true
```

baseline과 튜닝 결과 비교:

```bash
roslaunch ajou_control view_validation_runs.launch \
  input_csv:=$HOME/catkin_ws/src/ajou_control/validation_runs/max_steer_rate_08.csv \
  history_csvs:=$HOME/catkin_ws/src/ajou_control/validation_runs/baseline.csv \
  history_glob:= \
  enable_link_overlay:=true \
  enable_rviz:=true
```

RViz에서 확인할 display:

```text
MGeo Full Centerlines
MGeo Full Lane Edges
Validation Current and Previous Runs
```

표시 의미:

- 선택한 `input_csv`: 굵은 노란색 궤적
- `history_csvs`: 이전 비교 궤적
- MGeo link: 기준 지도 link

여러 궤적이 너무 많이 나오면 `history_glob`을 비운다.

```bash
history_glob:=
```

## Compare While Driving

새 튜닝 run을 실행하면서 baseline을 같이 보고 싶으면 `COMPARE_WITH`를 사용한다.

```bash
COMPARE_WITH=$HOME/catkin_ws/src/ajou_control/validation_runs/baseline.csv \
~/catkin_ws/src/ajou_control/scripts/run_mgeo_controller_tuning.sh max_steer_rate_08
```

이 경우 RViz에서 baseline 궤적과 현재 주행 궤적을 동시에 볼 수 있다.

## CSV Output

각 튜닝 케이스의 결과 CSV는 다음 위치에 저장된다.

```text
~/catkin_ws/src/ajou_control/validation_runs/<profile_name>.csv
```

CSV 주요 항목:

```text
ego_x, ego_y
path_x, path_y
lateral_error
heading_error_rad
steering_angle
lat_vel
long_vel
target_speed
ade
mean_ade
fde
lateral_rms
heading_rms
speed_rms
steer_saturation_rate
```

튜닝 판단 기준:

- `lateral_error`가 커지면 경로에서 좌우로 많이 벗어난다.
- `heading_error_rad`가 커지면 차량 방향이 경로 방향과 많이 다르다.
- `mean_ade`가 작을수록 전체적으로 경로를 잘 따라간다.
- `fde`는 현재 위치가 목표 path 끝점에서 얼마나 떨어져 있는지 나타낸다.
- `steer_saturation_rate`가 높으면 조향 제한에 자주 걸린다.

## rqt_multiplot Topics

validator는 rqt_multiplot에서 볼 수 있는 topic을 publish한다.

실행:

```bash
rosrun rqt_multiplot rqt_multiplot
```

추천 topic:

```text
/validation/lateral/lat_err
/validation/lateral/heading_err
/validation/lateral/steering_angle
/validation/lateral/lat_vel
/validation/lateral/lat_acc

/validation/longitudinal/long_vel
/validation/longitudinal/target_vel
/validation/longitudinal/long_vel_err
/validation/longitudinal/long_acc

/validation/metrics/mean_ade
/validation/metrics/fde
/validation/metrics/lateral_rms
/validation/metrics/heading_rms
/validation/metrics/speed_rms
/validation/metrics/progress
/validation/metrics/steer_saturation_rate
```

## Recommended Tuning Names

파일명은 파라미터 의미가 드러나게 짓는다.

예시:

```text
baseline
lookahead_7_16
lookahead_5_12
max_steer_rate_08
speed_2mps
max_steer_rad_030
```

각 이름은 다음 두 파일로 연결된다.

```text
configs/controller_tuning/<name>.yaml
validation_runs/<name>.csv
```

## Current Important Defaults

현재 기본적으로 맞춰둔 값:

```text
steering_sign = -1.0
velocity_scale = 0.27778
```

조향 방향 반대 문제는 `steering_sign=-1.0`으로 해결했다.
MORAI 속도 단위 문제는 `velocity_scale=0.27778`로 km/h를 m/s로 보정한다.
