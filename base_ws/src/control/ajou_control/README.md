# ajou_control

MORAI SIM에서 MGeo 기반 경로를 생성하고, Pure Pursuit + PI 제어기로 ego 차량을 추종시키며, 주행 결과를 CSV/RViz/rqt_multiplot으로 검증하기 위한 ROS1 Noetic 패키지입니다.

이 패키지는 `AjouNice2026/base_ws/src/control` 아래에서 사용합니다.

## 핵심 기능

- MGeo `link_set.json` 기반 주행 경로 생성
- Pure Pursuit 횡방향 제어
- PI 기반 종방향 속도 제어
- MORAI `/ctrl_cmd` publish
- 주행 결과 CSV 저장
- 이전 주행 궤적과 현재 주행 궤적 RViz 비교
- 제어기 튜닝 파라미터를 YAML 파일로 관리
- rqt_multiplot용 error/metric topic publish

## 전체 구조

```text
ajou_control
├── configs/controller_tuning/
│   └── baseline.yaml
├── launch/
│   ├── mgeo_path_stack.launch
│   ├── view_validation_runs.launch
│   └── tracking_validation.launch
├── scripts/
│   ├── run_mgeo_controller_tuning.sh
│   ├── mgeo_trajectory_generator.py
│   ├── pure_pursuit_pi_controller.py
│   ├── tracking_validator.py
│   └── validation_run_viewer.py
├── validation_runs/
│   └── .gitkeep
├── rviz/
│   └── mgeo_alignment.rviz
└── docs/
    ├── controller_tuning_workflow.md
    └── control_pipeline_status.md
```

## 빌드

```bash
cd ~/AjouNice2026/base_ws
catkin_make
source devel/setup.bash
```

개별 개발 환경이 `~/catkin_ws`라면 해당 workspace에서 동일하게 빌드하면 됩니다.

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 제어 파이프라인

```text
MGeo link_set.json
  -> mgeo_trajectory_generator.py
  -> /recorded_trajectory_path
  -> pure_pursuit_pi_controller.py
  -> /ctrl_cmd
  -> MORAI 주행
  -> tracking_validator.py
  -> validation_runs/<profile_name>.csv
  -> RViz / rqt_multiplot 비교
```

## 제어기 튜닝 방식

튜닝은 다음 원칙으로 관리합니다.

```text
튜닝 이름 = 제어기 파라미터 YAML 이름 = 주행 결과 CSV 이름
```

예를 들어 `baseline` 튜닝은 다음 두 파일이 한 세트입니다.

```text
configs/controller_tuning/baseline.yaml
validation_runs/baseline.csv
```

`baseline.yaml`은 제어기 파라미터를 저장하고, `baseline.csv`는 해당 파라미터로 주행한 결과를 저장합니다.

## 기본 튜닝 파일

기본 파라미터 파일:

```text
configs/controller_tuning/baseline.yaml
```

주요 값:

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
```

중요한 보정:

- `steering_sign: -1.0`: MORAI 조향 방향 반대 보정
- `velocity_scale: 0.27778`: MORAI 속도 단위가 km/h로 들어오는 경우 m/s로 변환
- `max_steer_rate`: 조향각 변화율 제한

## 튜닝 실행

baseline 실행:

```bash
source ~/AjouNice2026/base_ws/devel/setup.bash
~/AjouNice2026/base_ws/src/control/ajou_control/scripts/run_mgeo_controller_tuning.sh baseline
```

실행 결과:

```text
validation_runs/baseline.csv
```

같은 이름으로 다시 실행하면 같은 CSV가 최신 결과로 덮어써집니다.

## 새 튜닝 케이스 만들기

예시: `max_steer_rate_08` 케이스 생성

```bash
cd ~/AjouNice2026/base_ws/src/control/ajou_control

cp configs/controller_tuning/baseline.yaml \
   configs/controller_tuning/max_steer_rate_08.yaml
```

YAML 수정:

```bash
code configs/controller_tuning/max_steer_rate_08.yaml
```

실행:

```bash
source ~/AjouNice2026/base_ws/devel/setup.bash
~/AjouNice2026/base_ws/src/control/ajou_control/scripts/run_mgeo_controller_tuning.sh max_steer_rate_08
```

결과:

```text
validation_runs/max_steer_rate_08.csv
```

## 저장된 주행 궤적 보기

특정 주행 결과 하나만 RViz에서 보기:

```bash
roslaunch ajou_control view_validation_runs.launch \
  input_csv:=$HOME/AjouNice2026/base_ws/src/control/ajou_control/validation_runs/baseline.csv \
  history_glob:= \
  enable_link_overlay:=true \
  enable_rviz:=true
```

baseline과 튜닝 결과 비교:

```bash
roslaunch ajou_control view_validation_runs.launch \
  input_csv:=$HOME/AjouNice2026/base_ws/src/control/ajou_control/validation_runs/max_steer_rate_08.csv \
  history_csvs:=$HOME/AjouNice2026/base_ws/src/control/ajou_control/validation_runs/baseline.csv \
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
- `history_csvs`: 비교할 이전 주행 궤적
- MGeo link overlay: 기준 지도 link

## rqt_multiplot 확인 토픽

validator는 다음 토픽을 publish합니다.

```bash
rosrun rqt_multiplot rqt_multiplot
```

추천 토픽:

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

## CSV 저장 항목

`validation_runs/<profile_name>.csv`에는 다음 항목이 저장됩니다.

```text
time
nearest_index
progress
ego_x, ego_y
path_x, path_y
lateral_error
heading_error_rad
steering_angle
lat_vel, lat_acc
long_vel, target_speed, long_vel_error, long_acc
ade, mean_ade, fde
lateral_rms, heading_rms, speed_rms
steering_saturated
steer_saturation_rate
```

튜닝 판단 기준:

- `lateral_error`: 경로 좌우 오차
- `heading_error_rad`: 경로 방향과 차량 heading 차이
- `mean_ade`: 전체 평균 위치 오차
- `fde`: 현재 위치와 path 끝점 거리
- `steer_saturation_rate`: 조향 제한에 걸린 비율

## MGeo / RViz 좌표 보정

현재 기본 보정값은 launch에 반영되어 있습니다.

Control path 변환:

```text
offset_x = 134.24
offset_y = 1507.36
offset_z = -26.12
```

RViz preview / alignment 변환:

```text
preview_offset_x = 0.0
preview_offset_y = 560.0
preview_offset_z = -28.5
preview_ego_offset_x = -134.24
preview_ego_offset_y = -947.36
preview_ego_offset_z = -2.38
```

alignment만 확인:

```bash
roslaunch ajou_control mgeo_alignment_check.launch enable_rviz:=true
```

MGeo 경로 생성만 확인:

```bash
roslaunch ajou_control mgeo_path_stack.launch enable_rviz:=true
```

## 주요 토픽

| 토픽 | 타입 | 의미 |
| --- | --- | --- |
| `/recorded_trajectory_path` | `nav_msgs/Path` | 제어기가 추종하는 기준 경로 |
| `/mgeo_trajectory/preview_path` | `nav_msgs/Path` | RViz 확인용 preview 경로 |
| `/recorded_trajectory/target_speed` | `std_msgs/Float32` | 목표 속도 |
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` | MORAI 차량 제어 명령 |
| `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | MORAI ego 차량 상태 |
| `/validation/markers` | `visualization_msgs/MarkerArray` | 현재/이전 주행 궤적 비교 marker |

## 추가 문서

자세한 튜닝 절차:

```text
docs/controller_tuning_workflow.md
```

현재 파이프라인 구성 상태:

```text
docs/control_pipeline_status.md
```
