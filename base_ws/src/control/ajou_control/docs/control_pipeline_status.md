# Ajou Control Pipeline Status

작성 기준: `~/catkin_ws`

`~/AjouNice2026`는 GitHub 업로드/동기화용 작업물이므로 현재 실행 및 튜닝 기준에서 제외한다.
실제 제어 파이프라인 개발, 실행, 검증 기준은 `~/catkin_ws`이다.

## 현재 구축 상태

현재 파이프라인은 다음 흐름까지 연결되어 있다.

```text
MORAI / MGeo
  -> mgeo_trajectory_generator.py
  -> /recorded_trajectory_path
  -> pure_pursuit_pi_controller.py
  -> /ctrl_cmd
  -> tracking_validator.py
  -> validation CSV + RViz markers + rqt_multiplot topics
```

구성 패키지:

- `ajou_trajectory_recorder`: MORAI ego 주행 기록 및 CSV trajectory publish
- `ajou_control`: MGeo path 생성, Pure Pursuit + PI 제어, tracking validation
- `morai_msgs`: MORAI message dependency

빌드 확인:

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

`catkin_make`는 정상 통과한 상태이다.

## 핵심 실행 Launch

### MGeo 기반 전체 제어 스택

주 실행 파일:

```text
~/catkin_ws/src/ajou_control/launch/mgeo_path_stack.launch
```

포함 기능:

- MGeo link 기반 path 생성
- preview path RViz 표시
- ego/path 비교 marker 표시
- controller 선택 실행
- validator 선택 실행
- 이전 validation CSV overlay

기본 steering 방향 보정은 적용되어 있다.

```xml
<arg name="steering_sign" default="-1.0" />
```

즉 조향각 방향이 MORAI 기준과 반대로 들어가던 문제는 기본값에서 보정된 상태이다.

### Recorded CSV 기반 제어

관련 launch:

```text
~/catkin_ws/src/ajou_control/launch/recorded_path_stack.launch
~/catkin_ws/src/ajou_control/launch/recorded_path_control.launch
```

두 launch 모두 `steering_sign` 기본값이 `-1.0`으로 설정되어 있다.

## 현재 좌표 보정값

MGeo control path 생성 기본값:

```text
offset_x = 134.24
offset_y = 1507.36
offset_z = -26.12
ego_offset_x = 0.0
ego_offset_y = 0.0
ego_offset_z = 0.0
```

RViz preview / alignment 표시 기본값:

```text
preview_offset_x = 0.0
preview_offset_y = 560.0
preview_offset_z = -28.5
preview_ego_offset_x = -134.24
preview_ego_offset_y = -947.36
preview_ego_offset_z = -2.38
```

주의:

- control path와 preview/RViz overlay는 서로 다른 offset 계열을 사용한다.
- `mgeo_path_stack.launch`에서 controller/validator는 control path 기준 `/recorded_trajectory_path`를 사용한다.
- RViz에서 보는 preview overlay가 어긋나면 먼저 preview offset 문제인지, controller path offset 문제인지 분리해서 확인한다.

## 현재 권장 실행 명령

기본 조향 보정은 launch에 들어갔으므로 `steering_sign`은 생략 가능하다.
MORAI velocity가 km/h로 들어오는 문제는 아직 실행 인자로 보정한다.

```bash
source ~/catkin_ws/devel/setup.bash

roslaunch ajou_control mgeo_path_stack.launch \
  route_length_m:=120.0 \
  enable_controller:=true \
  enable_validator:=true \
  target_speed_mps:=2.0 \
  velocity_scale:=0.27778 \
  max_steer_rate:=0.8 \
  validation_csv:=$HOME/catkin_ws/src/ajou_control/validation_runs/run_001.csv \
  history_glob:=$HOME/catkin_ws/src/ajou_control/validation_runs/run_*.csv \
  enable_rviz:=true
```

## Controller 상태

제어기 파일:

```text
~/catkin_ws/src/ajou_control/scripts/pure_pursuit_pi_controller.py
```

입력:

- `/recorded_trajectory_path`
- `/recorded_trajectory/target_speed`
- `/Ego_topic`

출력:

- `/ctrl_cmd`
- `/pure_pursuit/debug_markers`

제어 방식:

- lateral: Pure Pursuit
- longitudinal: PI speed control

현재 중요한 보정:

- 조향 방향: `steering_sign=-1.0` 기본 적용
- 속도 단위: 실행 시 `velocity_scale:=0.27778` 적용 필요

## Validator 상태

검증 노드:

```text
~/catkin_ws/src/ajou_control/scripts/tracking_validator.py
```

기능:

- ego와 reference path의 nearest point 비교
- lateral error, heading error, ADE, FDE 계산
- steering, long/lat velocity, long/lat acceleration 기록
- CSV 저장
- RViz marker publish
- 이전 run CSV overlay
- rqt_multiplot용 Float32 topic publish

CSV 저장 항목:

```text
time
nearest_index
progress
ego_x
ego_y
path_x
path_y
lateral_error
heading_error_rad
steering_angle
lat_vel
lat_acc
long_vel
target_speed
long_vel_error
long_acc
ade
mean_ade
fde
lateral_rms
heading_rms
speed_rms
steering_saturated
steer_saturation_rate
```

현재 저장된 run 예시:

```text
~/catkin_ws/src/ajou_control/validation_runs/run_001_baseline.csv
~/catkin_ws/src/ajou_control/validation_runs/run_001_steer_sign_fixed.csv
~/catkin_ws/src/ajou_control/validation_runs/run_002.csv
~/catkin_ws/src/ajou_control/validation_runs/run_001.csv
```

## 주행 경로 저장 및 비교 방법

튜닝은 한 번에 하나의 파라미터만 바꾸고, 매 주행마다 CSV 파일명을 다르게 저장한다.

필요 기능은 두 단계로 나눈다.

1. 제어기 주행을 시작하고 해당 주행 궤적을 CSV로 기록한다.
2. 저장된 CSV를 나중에 다시 호출해서 RViz에서 이전/현재 주행 궤적으로 확인한다.

권장 실행 방식:

```bash
~/catkin_ws/src/ajou_control/scripts/run_mgeo_controller_tuning.sh baseline
```

위 명령을 실행하면 현재 주행은 자동으로 다음 형식의 CSV로 저장된다.

```text
~/catkin_ws/src/ajou_control/validation_runs/run_YYYYMMDD_HHMMSS_baseline.csv
```

다음 튜닝 주행은 이름만 바꿔서 실행한다.

```bash
~/catkin_ws/src/ajou_control/scripts/run_mgeo_controller_tuning.sh lookahead_test
```

이때 validator는 `validation_runs/run_*.csv`에 있는 이전 주행 기록을 읽어서 RViz의
`Validation Current and Previous Runs` display에 같이 표시한다.
즉 이전 주행 궤적과 새 튜닝 주행 궤적을 한 화면에서 비교할 수 있다.

저장된 특정 주행 궤적만 다시 호출해서 RViz에서 볼 때는 다음 launch를 사용한다.

```bash
source ~/catkin_ws/devel/setup.bash

roslaunch ajou_control view_validation_runs.launch \
  input_csv:=$HOME/catkin_ws/src/ajou_control/validation_runs/run_YYYYMMDD_HHMMSS_baseline.csv \
  history_glob:=$HOME/catkin_ws/src/ajou_control/validation_runs/run_*.csv \
  enable_rviz:=true
```

이 launch는 저장된 CSV를 읽어서 다음 토픽으로 publish한다.

```text
/validation/markers
/validation/selected_run_path
```

RViz에서는 `Validation Current and Previous Runs` display로 이전 run들과 선택한 run을 같이 본다.
선택한 run은 굵은 노란색 line, 이전 run들은 더 얇은 색상 line으로 표시된다.

좌표 보정 원칙:

- validation CSV의 `ego_x`, `ego_y`, `path_x`, `path_y`는 주행 당시 validator가 실제 비교에 사용한 좌표이다.
- 따라서 `mgeo_path_stack.launch`로 저장한 validation run은 기본적으로 추가 display offset 없이 RViz에 올리는 것이 맞다.
- 다른 좌표계에서 저장된 CSV를 불러오거나 RViz overlay가 어긋날 때만 `display_offset_x/y/z`를 사용한다.

예시:

```bash
roslaunch ajou_control view_validation_runs.launch \
  input_csv:=$HOME/catkin_ws/src/ajou_control/validation_runs/run_YYYYMMDD_HHMMSS_baseline.csv \
  history_glob:=$HOME/catkin_ws/src/ajou_control/validation_runs/run_*.csv \
  display_offset_x:=0.0 \
  display_offset_y:=0.0 \
  display_offset_z:=0.0 \
  enable_rviz:=true
```

파라미터를 바꿔서 실행할 때는 환경변수로 넘긴다.

```bash
TARGET_SPEED_MPS=2.0 \
MAX_STEER_RATE=0.8 \
ROUTE_LENGTH_M=120.0 \
~/catkin_ws/src/ajou_control/scripts/run_mgeo_controller_tuning.sh max_steer_rate_08
```

수동 실행 예시:

예시:

```bash
roslaunch ajou_control mgeo_path_stack.launch \
  route_length_m:=120.0 \
  enable_controller:=true \
  enable_validator:=true \
  target_speed_mps:=2.0 \
  velocity_scale:=0.27778 \
  max_steer_rate:=0.8 \
  validation_csv:=$HOME/catkin_ws/src/ajou_control/validation_runs/run_003_lookahead_7_16.csv \
  history_glob:=$HOME/catkin_ws/src/ajou_control/validation_runs/run_*.csv \
  enable_rviz:=true
```

비교 기준:

- RViz에서 이전 ego 주행 궤적과 현재 ego 주행 궤적을 overlay로 확인
- CSV에서 `lateral_error`, `heading_error_rad`, `mean_ade`, `fde` 확인
- rqt_multiplot에서 lateral/longitudinal time-series 확인

RViz 표시 설정:

```text
Controller Reference Path
  Topic: /recorded_trajectory_path

Validation Current and Previous Runs
  Topic: /validation/markers
```

`Validation Current and Previous Runs`에는 validator가 생성하는 다음 marker가 같이 표시된다.

- target path
- 현재 주행 ego path
- 이전 run ego path들
- 현재 ego 위치
- nearest target point
- ego-to-target error line
- validation summary text

이전 주행 궤적이 보이려면 실행할 때 `history_glob` 또는 `history_csvs`를 넘겨야 한다.
예시는 다음과 같다.

```bash
history_glob:=$HOME/catkin_ws/src/ajou_control/validation_runs/run_*.csv
```

추천 파일명 규칙:

```text
run_001_baseline.csv
run_002_steer_sign_fixed.csv
run_003_speed_2mps.csv
run_004_lookahead_7_16.csv
run_005_max_steer_rate_08.csv
```

핵심은 이전 run을 덮어쓰지 않는 것이다.
각 run을 남겨야 파라미터 하나를 바꿨을 때 trajectory와 error가 어떻게 변했는지 비교할 수 있다.

## rqt_multiplot 상태

설치 상태:

```bash
rospack find rqt_multiplot
```

현재 설치 확인됨:

```text
/opt/ros/noetic/share/rqt_multiplot
```

현재 `catkin_ws` 기준 상태:

- validator가 rqt_multiplot용 topic은 publish한다.
- rqt_multiplot 자동 XML preset은 아직 없다.
- 이전 `rqt_gui` 설정에는 `AjouNice2026` 쪽 XML 경로가 남아 있었지만 해당 파일은 존재하지 않는다.

수동 실행:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun rqt_multiplot rqt_multiplot
```

추천 plot group:

Lateral:

```text
/validation/lateral/lat_err
/validation/lateral/heading_err
/validation/lateral/steering_angle
/validation/lateral/lat_vel
/validation/lateral/lat_acc
```

Longitudinal:

```text
/validation/longitudinal/long_vel
/validation/longitudinal/target_vel
/validation/longitudinal/long_vel_err
/validation/longitudinal/long_acc
```

Run metrics:

```text
/validation/metrics/mean_ade
/validation/metrics/fde
/validation/metrics/lateral_rms
/validation/metrics/heading_rms
/validation/metrics/speed_rms
/validation/metrics/progress
/validation/metrics/steer_saturation_rate
```

다음 작업:

- `~/catkin_ws/src/ajou_control/config/rqt_multiplot/controller_tuning.xml` 추가
- rqt_multiplot을 해당 XML로 바로 여는 실행 스크립트 추가
- segfault 방지를 위해 plot 수는 한 번에 너무 많이 띄우지 않고 lateral / longitudinal / metrics 정도로 분리

## 남은 정리 작업

1. rqt_multiplot XML preset을 `catkin_ws` 기준으로 새로 저장한다.
2. run 저장용 launch arg 예시를 README에도 반영한다.
3. velocity scale도 MORAI 기준으로 계속 고정할지 결정한다.
4. 튜닝 run마다 CSV naming rule을 지킨다.
5. 안정화 후 `catkin_ws`의 정리된 작업물을 `AjouNice2026`로 옮긴다.
