# MORAI Control Pipeline 실행 방법

이 문서는 MORAI 시뮬레이터에서 차량을 연결하고 trajectory 생성·추종·기록·시각화하는 전체
실행 순서를 설명한다. ROS 명령은 특별히 표시하지 않는 한 **각각 별도 터미널**에서 실행한다.

## 표기 규칙

- **단축어**: `~/.bash_aliases`에 등록된 alias다. 입력이 짧지만 해당 PC에 alias 설정이
  적용되어 있어야 한다.
- **실제 명령어**: alias 없이 어느 터미널에서나 실행할 수 있는 원본 ROS 또는 shell
  명령이다. 다른 개발자 PC나 자동화 스크립트에서는 이 명령을 사용한다.
- **MORAI 조작**: 터미널 명령이 아니라 MORAI GUI에서 수행하는 F3/F4 조작이다.

예를 들어 다음 두 줄은 같은 기능이다.

```bash
# 단축어
viewer

# alias 없는 실제 명령어
roslaunch viewer viewer.launch
```

## 0. 최초 터미널 준비

새 터미널에서 alias가 동작하지 않으면 다음 환경을 먼저 적용한다.

```bash
source /opt/ros/noetic/setup.bash
source /home/graygorrila/AjouNice2026/base_ws/devel/setup.bash
source ~/.bash_aliases
```

간단히 다음 alias를 사용할 수도 있다.

```bash
so
sd
```

## 1. MORAI 및 ROS 연결

| 실행 순서 | 단축어·조작 | alias 없는 실제 명령어·설정 | 설명 |
| --- | --- | --- | --- |
| 1 | `rc` | `roscore` | ROS Master를 실행한다. 계속 켜 둔다. |
| 2 | `morai` | `bash ~/MORAI/MoraiLauncher_Lin/MORAISim.sh` | MORAI 시뮬레이터를 실행하고 사용할 시나리오를 연다. |
| 3 | MORAI에서 `F4` | `ego_network_1.json` 로드 | `configs/network`의 Ego UDP 네트워크 설정을 불러온다. |
| 4 | MORAI에서 `F4` | Ego Network의 UDP `Connect` | 차량 상태 수신과 제어 명령 송신용 Ego 네트워크를 연결한다. |
| 5 | MORAI에서 `F3` | `configs/sensors/Sensor_3.json` 로드 및 Connect | Camera, LiDAR, IMU, GPS 센서와 센서 네트워크를 활성화한다. |
| 6 | `udp` | `roslaunch sensor_interface morai_udp_interface.launch` | MORAI UDP 패킷과 ROS topic 사이의 변환 노드를 실행한다. |
| 7 | `rt` | `rostopic list` | Ego와 센서 topic이 생성됐는지 확인한다. |

대표 입력을 실제로 확인한다.

```bash
rostopic hz /Ego_topic
rostopic hz /sensor/gps
rostopic hz /sensor/imu
```

LiDAR와 Camera의 정확한 topic 이름은 다음 명령으로 확인한다.

```bash
rostopic list | grep -E 'lidar|camera|image'
```

## 2. Localization과 Viewer 실행

Viewer를 처음 사용하는 경우 화면별 의미와 상황별 명령은
[`control/view/README.md`](../base_ws/src/control/view/README.md)의 빠른 시작 안내를 먼저 본다.

| 실행 순서 | 단축어 | alias 없는 실제 명령어 | 설명 |
| --- | --- | --- | --- |
| 8 | `localization_morai_gt` | `roslaunch bringup localization_morai_gt.launch` | MORAI GT 차량 위치를 `/localization/kinematic_state`와 TF로 발행한다. |
| 9 | `viewer` | `roslaunch viewer viewer.launch` | Lanelet map, 현재 차량, 경로용 RViz와 실시간 rqt_plot을 실행한다. |

Localization 상태를 확인한다.

```bash
rostopic hz /localization/kinematic_state
rostopic echo -n 1 /localization/kinematic_state/header
rosrun tf tf_echo map base_link
```

정상 조건은 다음과 같다.

- `/localization/kinematic_state`가 지속적으로 발행된다.
- `header.frame_id`는 `map`, `child_frame_id`는 `base_link`다.
- RViz의 Fixed Frame은 `map`이다.
- RViz 지도 위에 주황색 차량 Marker가 보인다.

Viewer를 기능별로 따로 실행할 수도 있다.

| 단축어 | alias 없는 실제 명령어 | 기능 |
| --- | --- | --- |
| `viewer_rviz` | `roslaunch viewer viewer_rviz.launch` | 지도, 차량 위치, trajectory와 주행 궤적만 표시 |
| `viewer_rqt` | `roslaunch viewer viewer_rqt.launch` | 현재 제어값과 오차의 실시간 그래프만 표시 |
| `viewer_plot` | `roslaunch viewer viewer_plotjuggler.launch` | `compare_run.yaml`에서 선택한 저장 기록을 PlotJuggler로 분석 |

## 3. 기존 Trajectory로 주행

| 실행 순서 | 단축어 | alias 없는 실제 명령어 | 설명 |
| --- | --- | --- | --- |
| 10 | `traj_gen` | `roslaunch trajectory_generator trajectory_publisher.launch` | 이미 생성된 기본 trajectory CSV를 `/control/trajectory`로 발행한다. |
| 11 | `traj_val` | `roslaunch trajectory_validator record.launch` | 기록 대기 상태로 실행한다. Follower 입력 전에는 실제 기록을 시작하지 않는다. |
| 12 | `traj_fol` | `roslaunch trajectory_follower pp_pi.launch` | Pure Pursuit와 PI 제어기를 시작한다. 최초 유효 제어 입력부터 Validator 기록이 시작된다. |

`traj_val`을 `traj_fol`보다 먼저 실행하는 것이 권장 순서다. Follower는 시작 직후 제어를
시작하지만 Validator는 필요한 입력이 모두 들어오기 전까지 armed 상태로 기다린다.

실행 상태를 확인한다.

```bash
rostopic hz /control/trajectory
rostopic hz /control/command/steering
rostopic hz /ctrl_cmd
rostopic hz /trajectory_validator/debug
```

MORAI 차량이 움직이지 않으면 다음 항목을 확인한다.

```bash
rostopic info /ctrl_cmd
rostopic echo -n 1 /ctrl_cmd
```

- `/ctrl_cmd` publisher는 `longi_pi` 하나여야 한다.
- MORAI Ego Network가 `Connect` 상태여야 한다.
- UDP interface에서 control sender가 활성화되어 있어야 한다.
- 차량이 MORAI에서 수동 제어나 정지 상태로 잠겨 있지 않아야 한다.

## 4. 새 Trajectory 생성 후 주행

`traj_gen`은 trajectory를 새로 계산하는 명령이 아니라 **기존 CSV를 publish하는 명령**이다.
새 경로 또는 새 목표 속도를 만들려면 먼저 다음 YAML을 수정한다.

[trajectory_generator.yaml](../base_ws/src/control/trajectory_generator/config/trajectory_generator.yaml)

주요 설정:

```yaml
anchors:
  start_lanelet_id: 200889
  end_lanelet_id: 200258

trajectory:
  desired_velocity_kph: 20
  default_speed_limit_kph: 60.0
```

그다음 trajectory CSV를 생성한다.

```bash
rosrun trajectory_generator generate_trajectory.py \
  --config base_ws/src/control/trajectory_generator/config/trajectory_generator.yaml \
  --overwrite
```

생성 후 publish한다.

```bash
# 단축어
traj_gen

# alias 없는 실제 명령어
roslaunch trajectory_generator trajectory_publisher.launch
```

`desired_velocity_kph`는 원하는 주행 속도이고 `speed_limit_kph`는 넘어서는 안 되는 제한속도다.
최종 목표 속도는 두 값 중 작은 값을 m/s로 변환해 trajectory에 저장한다.

```text
velocity_mps = min(desired_velocity_kph, speed_limit_kph) / 3.6
```

속도 단위에 대한 자세한 내용은
[Control 속도 단위 규약](control_velocity_units.md)을 참고한다.

## 5. 기록 종료와 저장 위치

Validator는 목표 도착 또는 충돌 시 기록을 자동 종료한다. 필요하면 `traj_val` 터미널에서
`Ctrl-C`로 수동 종료할 수 있다.

기록은 다음 구조로 저장된다.

```text
base_ws/src/control/trajectory_validator/history/
└── <traj_sample>/
    └── <MMDD_HHMMSS_pp_pi>/
        ├── gt.csv
        ├── control_cycles.csv
        ├── vehicle_responses.csv
        └── metadata.json
```

- `gt.csv`: 추종해야 했던 기준 trajectory
- `control_cycles.csv`: 차량 상태, 목표값, 제어 명령과 추종 오차
- `vehicle_responses.csv`: 제어 명령 이후 차량 반응
- `metadata.json`: 실행 설정, 파일 hash, 통계와 종료 원인

최근 기록을 확인한다.

```bash
find base_ws/src/control/trajectory_validator/history \
  -mindepth 2 -maxdepth 2 -type d | sort | tail
```

## 6. 저장 기록 Replay·Final·Compare

### Replay와 Final 기록 선택

[history_run.yaml](../base_ws/src/control/trajectory_validator/config/history_run.yaml)을 수정한다.

```yaml
traj_sample: "200889_200258"
run_id: "0719_172905_pp_pi"
```

| 단축어 | alias 없는 실제 명령어 | 기능 | 출력 |
| --- | --- | --- | --- |
| `traj_val_replay` | `roslaunch trajectory_validator history.launch mode:=replay` | 기록 당시 시간 순서대로 주행을 재생 | `/trajectory_validator/*` |
| `traj_val_final` | `roslaunch trajectory_validator history.launch mode:=final` | 저장된 기준·실제 전체 경로를 즉시 표시 | `/trajectory_validator/*` |
| `traj_val_plot` | 아래 별도 명령 참고 | 같은 기록의 CSV를 PlotJuggler로 분석 | PlotJuggler |

`traj_val_plot`의 alias 없는 명령은 다음과 같다.

```bash
roslaunch viewer viewer_plotjuggler.launch \
  config_file:=$(rospack find trajectory_validator)/config/history_run.yaml
```

### Compare 기록 선택

[compare_run.yaml](../base_ws/src/control/trajectory_validator/config/compare_run.yaml)을 수정한다.

```yaml
traj_sample: "200889_200258"
run_id: "0719_172905_pp_pi"
```

| 단축어 | alias 없는 실제 명령어 | 기능 | 출력 |
| --- | --- | --- | --- |
| `traj_val_compare` | `roslaunch trajectory_validator history.launch mode:=compare` | 선택한 과거 전체 경로를 현재 경로와 겹쳐 표시 | `/trajectory_compare/*` |
| `viewer_plot` | `roslaunch viewer viewer_plotjuggler.launch` | 같은 Compare 기록의 CSV를 PlotJuggler로 분석 | PlotJuggler |

Replay/Final용 `history_run.yaml`과 Compare용 `compare_run.yaml`은 서로 독립적이다.

## 7. 권장 전체 실행 순서 요약

| 순서 | 실행 위치 | 단축어·조작 | alias 없는 실제 명령어·설정 | 확인 대상 |
| --- | --- | --- | --- | --- |
| 1 | 터미널 1 | `rc` | `roscore` | ROS Master |
| 2 | 터미널 또는 MORAI | `morai` | `bash ~/MORAI/MoraiLauncher_Lin/MORAISim.sh` 실행 후 시나리오 로드 | 차량 초기 위치 |
| 3 | MORAI | `F4` | `configs/network/ego_network_1.json` 로드 후 UDP Connect | Ego 통신 |
| 4 | MORAI | `F3` | `configs/sensors/Sensor_3.json` 로드 후 Connect | 센서 통신 |
| 5 | 터미널 2 | `udp` | `roslaunch sensor_interface morai_udp_interface.launch` | `/Ego_topic`, 센서 topic |
| 6 | 터미널 3 | `localization_morai_gt` | `roslaunch bringup localization_morai_gt.launch` | kinematic state와 TF |
| 7 | 터미널 4 | `viewer` | `roslaunch viewer viewer.launch` | 지도와 차량 위치 |
| 8 | 터미널 5 | `traj_gen` | `roslaunch trajectory_generator trajectory_publisher.launch` | 기준 trajectory |
| 9 | 터미널 6 | `traj_val` | `roslaunch trajectory_validator record.launch` | Validator armed 상태 |
| 10 | 터미널 7 | `traj_fol` | `roslaunch trajectory_follower pp_pi.launch` | 조향·가감속 제어와 기록 시작 |

종료는 제어 안전을 위해 역순으로 진행한다.

1. `traj_fol` 종료
2. 차량 정지 확인
3. 필요하면 `traj_val` 종료 및 기록 저장 확인
4. Viewer와 localization 종료
5. UDP interface 종료
6. MORAI 종료
7. `roscore` 종료

## 8. 빠른 상태 점검

```bash
rostopic hz /Ego_topic
rostopic hz /localization/kinematic_state
rostopic hz /control/trajectory
rostopic hz /ctrl_cmd
rosnode list | grep -E 'morai|trajectory|viewer|current_vehicle'
```

네 핵심 topic 중 하나라도 나오지 않으면 그 topic을 생성하는 앞 단계부터 확인한다.
