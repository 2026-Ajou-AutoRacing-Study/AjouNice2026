# trajectory_generator 사용 가이드

`trajectory_generator`는 **Lanelet 지도에서 제어기가 따라갈 기준 trajectory를 만들고,
그 결과를 ROS topic으로 반복 발행하는 패키지**다.

처음 사용할 때는 아래 두 작업을 구분해야 한다.

```text
[오프라인 생성: 경로/속도를 바꿀 때 한 번]
지도 + drive-course + 시작/종료 lanelet + 목표 속도
  -> generate_trajectory.py
  -> traj_sample/<start>_<end>.csv

[온라인 발행: 주행할 때마다 실행]
traj_sample/*.csv
  -> trajectory_publisher_node
  -> /control/trajectory
  -> trajectory_follower
```

즉, `traj_gen`은 이름과 달리 새 CSV를 생성하지 않는다. 이미 생성된 CSV를 읽어
`/control/trajectory`로 발행하는 단축 명령이다.

## 가장 빠른 사용 순서

### 1. 최초 1회 빌드

저장소 루트에서 실행한다.

```bash
cd base_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps trajectory_generator
source devel/setup.bash
cd ..
```

새 터미널을 열 때마다 다음 환경 적용은 다시 필요하다.

```bash
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
```

`--only-pkg-with-deps`는 catkin package whitelist를 유지한다. 전체 workspace를 다시 빌드할
때는 `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`로 해제한다.

### 2. 기존 기본 trajectory 발행

경로를 새로 만들 필요가 없다면 바로 실행한다.

```bash
roslaunch trajectory_generator trajectory_publisher.launch
```

공용 alias를 적용한 환경에서는 같은 명령이 `traj_gen`이다. 기본 파일은
`traj_sample/200889_200258.csv`이며, 노드 시작 시 한 번 읽힌다.

다른 CSV를 발행하려면 package 내부 상대경로를 넘긴다.

```bash
roslaunch trajectory_generator trajectory_publisher.launch \
  trajectory_csv:=traj_sample/200889_200348.csv
```

발행 중 CSV를 수정해도 실행 중인 trajectory는 바뀌지 않는다. 생성 또는 수정 후에는
publisher를 재시작한다.

### 3. 새 trajectory 생성

경로 구간이나 목표 속도를 바꿀 때만 이 단계가 필요하다. 공유 설정 파일을 직접 바꾸기보다
Git에서 제외되는 `config/trajectory_generator.local.yaml`을 복사해 개인 실험에 사용한다.

```bash
cp -n base_ws/src/control/trajectory_generator/config/trajectory_generator.yaml \
  base_ws/src/control/trajectory_generator/config/trajectory_generator.local.yaml
```

로컬 YAML에서 다음 값을 확인한다.

```yaml
anchors:
  start_lanelet_id: 200889
  end_lanelet_id: 200258

trajectory:
  resample_interval_m: 0.1
  desired_velocity_kph: 20
  default_speed_limit_kph: 60.0
```

생성 명령은 저장소 루트에서 실행한다.

```bash
rosrun trajectory_generator generate_trajectory.py \
  --config base_ws/src/control/trajectory_generator/config/trajectory_generator.local.yaml
```

결과는 설정한 ID에 따라 자동으로 저장된다.

```text
base_ws/src/control/trajectory_generator/traj_sample/<start_lanelet_id>_<end_lanelet_id>.csv
```

### K-City v44 competition samples

`config/samples/`에는 competition route publisher의 시작 preset과 대응하는 생성 설정을 둔다.
모두 안전한 초기 비교용 `20 km/h` 상한이며, map의 더 낮은 제한속도는 그대로 적용된다.

| preset | 생성 설정 | 출력 CSV | course position |
| --- | --- | --- | --- |
| full | `config/samples/competition_full.yaml` | `traj_sample/200889_200892.csv` | 공식 start → finish loop |
| early_middle | `config/samples/competition_early_middle.yaml` | `traj_sample/200311_200892.csv` | 약 35% → finish |
| middle | `config/samples/competition_middle.yaml` | `traj_sample/201086_200892.csv` | 약 50% → finish |
| late | `config/samples/competition_late.yaml` | `traj_sample/200360_200892.csv` | 약 75% → finish |

코너 추종·가변 lookahead 튜닝에는 아래처럼 코너 전후 margin을 포함한 sample을 사용한다.

| sample | 생성 설정 | 출력 CSV | 포함 구간 |
| --- | --- | --- | --- |
| start left-right | `config/samples/corner_start_left_right.yaml` | `traj_sample/200889_200287.csv` | 출발 좌회전 → 우회전, 22 m exit |
| left S curve | `config/samples/corner_left_s_curve.yaml` | `traj_sample/200287_200292.csv` | 22 m entry → 연속 좌커브 → 50 m exit |
| outer turn sequence | `config/samples/corner_outer_turn_sequence.yaml` | `traj_sample/200950_200311.csv` | right/right/left/left, 긴 우커브 포함 |
| roundabout | `config/samples/corner_roundabout.yaml` | `traj_sample/200968_200101.csv` | 우커브 진입 → 회전교차로 → 167 m exit |
| return tight rights | `config/samples/corner_return_tight_rights.yaml` | `traj_sample/201399_200892.csv` | 우회전 연속·타이트 우커브 → finish |

예를 들어 full sample은 다음처럼 생성·발행한다.

```bash
rosrun trajectory_generator generate_trajectory.py \
  --config base_ws/src/control/trajectory_generator/config/samples/competition_full.yaml
roslaunch trajectory_generator trajectory_publisher.launch \
  trajectory_csv:=traj_sample/200889_200892.csv
```

같은 이름의 파일이 이미 있으면 실수로 기존 결과를 잃지 않도록 생성이 실패한다. 의도적으로
교체할 때만 `--overwrite`를 추가한다.

```bash
rosrun trajectory_generator generate_trajectory.py \
  --config base_ws/src/control/trajectory_generator/config/trajectory_generator.local.yaml \
  --overwrite
```

새로 생성한 CSV를 발행할 때는 2단계의 `trajectory_csv` 인자로 해당 파일을 선택한다. 새 CSV는
기본적으로 Git에서 제외되며, 팀 공용 sample을 갱신할 때만 별도로 합의해 반영한다.

## 설정값 이해하기

| 설정 | 사용자가 정하는 내용 | 주의사항 |
| --- | --- | --- |
| `map.lanelet_map_file` | 중심선을 읽을 Lanelet2 OSM | drive-course와 동일한 map version이어야 한다. |
| `map.projector_file` | 위경도를 `map` 좌표로 바꾸는 projector | 현재 `LocalCartesianUTM`만 지원한다. |
| `map.route_lanelets_file` | lanelet 주행 순서와 successor 관계 | `role=preferred_course` 행을 사용한다. |
| `reference.global_path_file` | 중심선 진행 방향 판별용 기준 경로 | 최소 2개 점이 필요하다. |
| `anchors.start_lanelet_id` | 생성 구간의 시작 lanelet | end까지 유일한 successor chain이 있어야 한다. |
| `anchors.end_lanelet_id` | 생성 구간의 종료 lanelet | 출력 파일명에도 사용된다. |
| `anchors.start_route_order` | 동일 lanelet ID가 반복될 때 시작 occurrence | 생략하면 모든 start 후보를 검사한다. |
| `anchors.end_route_order` | 동일 lanelet ID가 반복될 때 종료 occurrence | 한 바퀴 loop처럼 start/end ID가 같을 때 필요하다. |
| `trajectory.resample_interval_m` | trajectory 점 간격 | 현재 계약상 `0.1` m 고정이다. |
| `trajectory.desired_velocity_kph` | 차량에 요구할 순항 속도 | 양수이며 단위는 km/h다. |
| `trajectory.default_speed_limit_kph` | 지도에 제한속도가 없을 때의 안전 상한 | 목표 속도가 아니라 fallback 제한속도다. |
| `trajectory.speed_profile` | lanelet별 속도와 가감속 envelope | 생략하면 기존 고정속도 생성 동작을 유지한다. |

최종 CSV 속도는 다음과 같이 저장된다.

```text
velocity_mps = min(desired_velocity_kph, 적용 제한속도 kph) / 3.6
```

적용 제한속도의 우선순위는 `drive-course CSV -> Lanelet2 OSM ->
default_speed_limit_kph`다. 출발/종료 감속 ramp는 아직 생성하지 않으며 마지막 점도 자동으로
`0 m/s`가 되지 않는다. 속도 단위 전체 계약과 MORAI GT 입력은
[Control 속도 단위 규약](../../../../docs/control_velocity_units.md)을 참고한다.

### Lanelet별 가변속도 profile

`trajectory.speed_profile`을 지정하면 lanelet별 속도 상한을 만든 뒤, 0.1 m 경로를 전방·후방으로
통과하며 최대 가속도와 감속도를 만족하는 연속 속도열을 생성한다.

```yaml
anchors:
  start_lanelet_id: 200889
  end_lanelet_id: 200889
  start_route_order: 1
  end_route_order: 42

trajectory:
  output_csv: base_ws/src/control/trajectory_generator/traj_sample/variable/200889_200889_v.csv
  speed_profile:
    default_velocity_kph: 60.0
    respect_speed_limits: false
    max_acceleration_mps2: 2.0
    max_deceleration_mps2: 2.5
    lanelet_velocity_kph:
      200392: 30.0
      200101: 140.0
```

`respect_speed_limits: true`이면 lanelet 속도는 기존 지도/drive-course 제한으로 다시 잘린다.
`false`는 실험용 profile이 지도 제한보다 우선한다는 명시적 override이므로 폐쇄된 검증 환경에서만
사용한다. 완성된 full-lap 예시는
[`config/trajectory_generator_200889_full_variable.yaml`](config/trajectory_generator_200889_full_variable.yaml)이다.

## 정상 동작 확인

생성 성공 시 터미널에는 출력 파일, lanelet 수, 경로 길이, 점 개수, 실제 속도 범위가 표시된다.
publisher 실행 후에는 다음을 확인한다.

```bash
rostopic hz /control/trajectory
rostopic echo -n 1 /control/trajectory/header
```

기본 발행 주기는 50 Hz이고 `frame_id`는 `map`이다. RViz에서는 다음 debug topic을 추가한다.

- `/control/debug/generator/path` (`nav_msgs/Path`): 전체 기준 경로
- `/control/debug/generator/markers` (`visualization_msgs/MarkerArray`): 경로와 시작/종료점

## 자주 만나는 오류

| 메시지 또는 증상 | 원인과 조치 |
| --- | --- |
| `output exists; pass --overwrite` | 같은 start/end CSV가 이미 있다. 보존할 파일인지 확인한 뒤에만 `--overwrite`를 쓴다. |
| `no successor chain reaches ...` | start/end ID가 drive-course에서 연결되지 않는다. map version과 anchor ID를 확인한다. |
| `anchors are ambiguous` | start에서 end까지 가능한 chain이 둘 이상이다. 더 명확한 anchor 구간을 선택한다. |
| `lanelet ... has no usable centerline` | OSM에 해당 lanelet의 유효한 centerline이 없거나 map version이 다르다. |
| `starts ... m from previous centerline` | 연속 lanelet 중심선 사이가 5 m보다 멀다. route와 OSM의 조합을 확인한다. |
| publisher가 CSV를 열지 못함 | `trajectory_csv`는 package 내부 상대경로여야 하며 절대경로나 `..`를 쓸 수 없다. |
| YAML/CSV를 바꿨는데 경로가 그대로임 | publisher는 시작할 때 CSV를 한 번만 읽는다. CSV를 다시 생성한 뒤 노드를 재시작한다. |

## 패키지의 책임 범위

이 패키지는 정적 기준 경로를 만들고 발행한다. 차량의 현재 위치를 보며 경로를 다시 계획하거나,
장애물을 회피하거나, 조향·가감속 명령을 직접 만들지는 않는다. 실제 추종 제어는
`trajectory_follower`, 주행 결과 기록과 평가는 `trajectory_validator`의 책임이다.

내부 생성 알고리즘, CSV 및 ROS interface 계약은 [trajectory_gen.md](trajectory_gen.md)를
참고한다. 전체 제어 파이프라인 실행 순서는
[제어 파이프라인 실행 가이드](../../../../docs/how_to_launch_control_pipeline.md)를 참고한다.
