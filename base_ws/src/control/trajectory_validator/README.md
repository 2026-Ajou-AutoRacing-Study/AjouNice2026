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

`traj_val`은 현재 generator trajectory와 같은 `traj_sample`의 완료된 최신 3개 run도
함께 연다. 기준 경로·pose·debug는 재발행하지 않고 실제 주행 path만 아래 토픽에 latch
발행한다. `his1`이 가장 최근 기록이다.

```text
/trajectory_validator/ego_path/his1
/trajectory_validator/ego_path/his2
/trajectory_validator/ego_path/his3
```

현재 주행은 기존 `/trajectory_validator/ego_path` 그대로다. history가 3개보다 적으면
존재하는 기록만 표시한다.

RViz와 rqt는 `viewer` 패키지가 전담한다. 별도 터미널에서 `roslaunch viewer viewer.launch`를
실행한다. `trajectory_publisher.launch`가 실행되어 기준 경로를 제공하면
Validator는 Follower보다 먼저 실행해도 된다. `/purepursuit_autoware`, `/longi_pi_autoware`, `/ctrl_cmd`가 아직
없으면 종료하지 않고 armed 상태로 기다린다. Follower가 시작되어 최초 유효한 제어 입력이
들어오는 순간 실제 실행 파라미터를 자동 수집한다.
노드는 먼저 armed 상태로 대기하며 trajectory, kinematic state, Ego 상태, `/ctrl_cmd`가 모두
fresh하고 동기화된 최초 시점을 기록의 `t=0`으로 사용한다. 기록 시작 후 발생하는 일시적인
입력 누락이나 상태-명령 연결 실패는 `control_pair_valid=false`로 저장한다.
충돌 또는 목표 도착 시 자동 종료하며, 수동 종료는 `Ctrl-C`로 한다. 목표 도착은 마지막
점과의 거리(`goal_tolerance_m`)뿐 아니라 마지막 station 전 `goal_progress_margin_m` 구간에
진입했는지도 함께 확인한다. 따라서 시작·종료 좌표가 같은 폐루프 경로도 출발 직후 종료되지
않고, 한 바퀴를 진행한 뒤에만 goal로 종료된다.
결과는 자동 생성된 run ID로 저장된다.

```text
history/<traj_sample>/<MMDD_HHMMSS_pp_autoware>/tracking_samples.csv
history/<traj_sample>/<MMDD_HHMMSS_pp_autoware>/metadata.json
history/<traj_sample>/<MMDD_HHMMSS_pp_autoware>/summary.svg
```

`summary.svg`는 competition route와 같은 thin-line SVG 포맷으로 Lanelet map(회색), target/GT path(초록),
실제 ego path(파랑), GT 시작/종료점을 한 장에 표시한다. map은 `summary_svg.map_file`과
`summary_svg.projector_file`로 고정하며, 사용한 파일과 hash는 `metadata.json`의 `map`에,
SVG hash와 점 수는 `artifacts.summary_svg`에 저장한다. ego sample이 하나도 없는 run도 GT와
map을 남기고 ego path는 생략한다.

저장 결과를 확인한다.

```bash
find base_ws/src/control/trajectory_validator/history/200889_200258 \
  -maxdepth 2 -type f | sort
```

기존 CSV history에도 이미지를 추가하려면 다음을 실행한다. 다른 버전의 지도에서 만든
run은 그 지도 파일을 `--map-file`로 지정한다.

```bash
rosrun trajectory_validator render_history_summary.py \
  --traj-sample 200889_200258 \
  --run-id 0725_151247_pp_autoware
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

schema v5는 `tracking_samples.csv`, trajectory별로 한 번만 보존되는
`history/<traj_sample>/reference_<hash>.csv`, 각각의 hash를 검사한다.
고주기 CSV에는 위치·속도·yaw rate, LD, 경로/목표 곡률, 횡/헤딩 오차, 명령/실제 조향각,
유효성만 저장한다. 고정 controller/path-processing 설정은 `metadata.json`에 한 번만 기록한다.
`ego_velocity_mps`는 LD 계산에 사용되는 signed base-link longitudinal velocity다.
기존 schema v3/v4 run도 replay/final path로 계속 읽을 수 있다.

Compare는 `/trajectory_compare/gt_path`, `/trajectory_compare/ego_path`,
`/trajectory_compare/ego_pose`를 사용하므로 record/replay 표시를 덮지 않는다. Viewer RViz에는
보라색 Compare GT와 주황색 Compare 실제 주행 Path가 기본 등록되어 있다.

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
  50 Hz로 기록한다.
- `tracking_samples.csv`에는 분석에 필요한 21개 시계열만 저장한다. 기준 trajectory는
  `metadata.json`의 원본 경로와 hash로 참조한다.
- 저장 run 하나를 `TrajectoryDebug`, Path, Pose, Marker topic으로 복원한다.
- trajectory나 제어 명령을 수정하거나 생성하지 않는다.

| 입력 | 출력 |
| --- | --- |
| `/control/trajectory`, `/localization/kinematic_state`, `/Ego_topic`, `/ctrl_cmd`, `/CollisionData` | 분리된 history run |
| history CSV/JSON | `/trajectory_validator/*` debug/visualization topic |

설정은 `config/trajectory_validator.yaml`, 데이터 계약과 metric 정의는
[trajectory_val.md](trajectory_val.md)를 참고한다.
