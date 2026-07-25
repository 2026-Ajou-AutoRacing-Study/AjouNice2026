# trajectory_validator

속도 단위의 전체 계약과 MORAI GT 입력 주의사항은
[`docs/control_velocity_units.md`](../../../../docs/control_velocity_units.md)를 참고한다.

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

MORAI GT localization의 속도 입력은 `config/trajectory_validator.yaml`의
`input/kinematic_velocity_scale_to_mps`를 적용해 km/h에서 m/s로 정규화한 후 기록한다.
따라서 `*_mps` CSV 열과 속도 오차는 실제 m/s 값이다. 이미 m/s를 출력하는 localization을
사용할 때는 이 scale을 `1.0`으로 변경한다.

## 사용법

### 1. 현재 주행 기록

```bash
roslaunch trajectory_validator record.launch
```

기본 sample과 위 인자를 사용하는 단축 명령은 다음과 같다.

```bash
traj_val
```

RViz와 rqt는 `viewer` 패키지가 전담한다. 별도 터미널에서 `roslaunch viewer viewer.launch`를
실행한다. `trajectory_publisher.launch`가 실행되어 기준 경로를 제공하면
Validator는 Follower보다 먼저 실행해도 된다. `/purepursuit`, `/longi_pi`, `/ctrl_cmd`가 아직
없으면 종료하지 않고 armed 상태로 기다린다. Follower가 시작되어 최초 유효한 제어 입력이
들어오는 순간 실제 실행 파라미터를 자동 수집한다.
노드는 먼저 armed 상태로 대기하며 trajectory, kinematic state, Ego 상태, `/ctrl_cmd`가 모두
fresh하고 동기화된 최초 시점을 기록의 `t=0`으로 사용한다. 기록 시작 후 발생하는 일시적인
입력 누락이나 상태-명령 연결 실패는 `control_pair_valid=false`, 명령-반응 연결 실패는
`response_pair_valid=false`로 각각 저장한다.
충돌 또는 목표 도착 시 자동 종료하며, 수동 종료는 `Ctrl-C`로 한다.
결과는 자동 생성된 run ID로 저장된다.

```text
history/<traj_sample>/<MMDD_HHMMSS_pp_pi>/gt.csv
history/<traj_sample>/<MMDD_HHMMSS_pp_pi>/control_cycles.csv
history/<traj_sample>/<MMDD_HHMMSS_pp_pi>/vehicle_responses.csv
history/<traj_sample>/<MMDD_HHMMSS_pp_pi>/metadata.json
history/<traj_sample>/<MMDD_HHMMSS_pp_pi>/summary.json
```

저장 결과를 확인한다.

```bash
find base_ws/src/control/trajectory_validator/history/200889_200258_20kph \
  -maxdepth 2 -type f | sort
```

### 2. 저장 run 호출

Replay와 Final path는 `config/history_run.yaml`에서 열어볼 기록을 선택한다. Compare와
PlotJuggler는 별도의 비교 기준인 `config/compare_run.yaml`을 사용한다.

```bash
roslaunch trajectory_validator history.launch mode:=replay
roslaunch trajectory_validator history.launch mode:=final
roslaunch trajectory_validator history.launch mode:=compare
```

- `replay`: 기록을 시간순으로 `/trajectory_validator` 토픽에 재생한다.
- `final`: 저장된 전체 경로를 즉시 `/trajectory_validator` 토픽에 latch 발행한다.
- `compare`: 저장된 전체 경로를 즉시 `/trajectory_compare` 토픽에 latch 발행한다.

모든 모드는 `gt.csv`, `control_cycles.csv`, `vehicle_responses.csv`, `metadata.json`과 hash를
검사한다.

Compare는 `/trajectory_compare/gt_path`, `/trajectory_compare/ego_path`,
`/trajectory_compare/ego_pose`를 사용하므로 record/replay 표시를 덮지 않는다. Viewer RViz에는
빨간색 Compare GT와 Compare 실제 주행 Path가 기본 등록되어 있다.

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
- `gt.csv`에는 GT point, `control_cycles.csv`에는 상태→명령 연결,
  `vehicle_responses.csv`에는 명령→차량 반응을 저장한다. `metadata.json`에는 설정·hash·통계와
  종료 원인을 저장한다. `summary.json`에는 `±0.3 m` 횡오차 유지율, 횡오차 P95,
  곡률 구간별 횡·종방향·조향 오차, 속도 오버슈트, 종방향 속도·가속도와 조향 변화율을
  저장한다.
- 저장 run 하나를 `TrajectoryDebug`, Path, Pose, Marker topic으로 복원한다.
- trajectory나 제어 명령을 수정하거나 생성하지 않는다.

| 입력 | 출력 |
| --- | --- |
| `/control/trajectory`, `/localization/kinematic_state`, `/Ego_topic`, `/ctrl_cmd`, `/CollisionData` | 분리된 history run |
| history CSV/JSON | `/trajectory_validator/*` debug/visualization topic |

설정은 `config/trajectory_validator.yaml`, 데이터 계약과 metric 정의는
[trajectory_val.md](trajectory_val.md)를 참고한다.
