# trajectory_validator

## 실행 준비

```bash
cd /home/jihun/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps trajectory_validator
source devel/setup.bash
cd /home/jihun/AjouNice2026
```

`--only-pkg-with-deps`는 package whitelist를 유지한다. 전체 workspace를 다시 구성할 때는
`catkin_make -DCATKIN_WHITELIST_PACKAGES=""`로 해제한다.

녹화 전 다음 topic을 확인한다.

```bash
rostopic hz /control/trajectory
rostopic hz /localization/kinematic_state
rostopic hz /Ego_topic
rostopic hz /ctrl_cmd
```

## 사용법

### 1. 현재 주행 녹화

```bash
roslaunch trajectory_validator trajectory_validator.launch \
  input_trajectory_csv:=base_ws/src/control/trajectory_generator/traj_sample/200889_200258.csv \
  run_params:=pp_pi_ld4p0 \
  map_version:=v44
```

기본 sample과 위 인자를 사용하는 단축 명령은 다음과 같다.

```bash
traj_val
```

`input_trajectory_csv`, `run_params`, `map_version`은 필수다. 종료는 `Ctrl-C`로 한다.
결과는 자동 생성된 run ID로 저장된다.

```text
history/<traj_sample>/<MMDD_HHMMSS_run_params>.csv
history/<traj_sample>/<MMDD_HHMMSS_run_params>.json
```

저장 결과를 확인한다.

```bash
find base_ws/src/control/trajectory_validator/history/200889_200258 \
  -maxdepth 1 -type f | sort
```

### 2. 저장 run 재생

정확한 traj sample과 run ID를 지정한다.

```bash
rosrun trajectory_validator publish_history.py \
  --traj-sample 200889_200258 \
  --run-id 0717_143012_pp_pi_ld4p0
```

재생은 저장된 sample을 30 Hz로 한 번 publish하고 종료한다. CSV/JSON pair가 없거나 hash가
다르면 실행하지 않는다.

### 3. RViz와 rqt_plot 확인

RViz 표시 topic:

- `/trajectory_validator/gt_path`
- `/trajectory_validator/ego_path`
- `/trajectory_validator/ego_pose`
- `/trajectory_validator/markers`

주요 오차를 plot한다.

```bash
rqt_plot \
  /trajectory_validator/debug/lateral_error_m \
  /trajectory_validator/debug/heading_error_rad \
  /trajectory_validator/debug/velocity_error_mps
```

## 기능과 역할

- 현재 주행의 GT trajectory, map-frame kinematic state, MORAI telemetry, `/ctrl_cmd`, 오차 지표를
  30 Hz로 기록한다.
- CSV에는 GT point와 시간순 sample을, JSON에는 설정·hash·통계·종료 원인을 저장한다.
- 저장 run 하나를 `TrajectoryDebug`, Path, Pose, Marker topic으로 복원한다.
- trajectory나 제어 명령을 수정하거나 생성하지 않는다.

| 입력 | 출력 |
| --- | --- |
| `/control/trajectory`, `/localization/kinematic_state`, `/Ego_topic`, `/ctrl_cmd`, `/CollisionData` | history CSV/JSON |
| history CSV/JSON | `/trajectory_validator/*` debug/visualization topic |

설정은 `config/trajectory_validator.yaml`, 데이터 계약과 metric 정의는
[trajectory_val.md](trajectory_val.md)를 참고한다.
