# trajectory_generator

## 실행 준비

저장소의 `base_ws`에서 패키지와 의존성을 빌드한 뒤 환경을 적용한다.

```bash
cd /home/jihun/AjouNice2026/base_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps trajectory_generator
source devel/setup.bash
cd /home/jihun/AjouNice2026
```

`--only-pkg-with-deps`는 catkin package whitelist를 유지한다. 나중에 전체 workspace를 다시
구성할 때는 `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`로 해제한다.

## 사용법

### 1. Trajectory 생성

먼저 [config/trajectory_generator.yaml](config/trajectory_generator.yaml)에서 map, drive-course,
global path, 시작/종료 lanelet ID를 확인한다.

```bash
rosrun trajectory_generator generate_trajectory.py \
  --config base_ws/src/control/trajectory_generator/config/trajectory_generator.yaml
```

결과는 다음 규칙으로 저장된다.

```text
base_ws/src/control/trajectory_generator/traj_sample/
  <start_lanelet_id>_<end_lanelet_id>.csv
```

같은 파일을 다시 만들 때만 명시적으로 덮어쓴다.

```bash
rosrun trajectory_generator generate_trajectory.py \
  --config base_ws/src/control/trajectory_generator/config/trajectory_generator.yaml \
  --overwrite
```

### 2. Trajectory publish

기본 sample을 publish한다.

```bash
roslaunch trajectory_generator trajectory_publisher.launch
```

등록된 단축 명령은 다음과 같다.

```bash
traj_gen
```

다른 sample은 package 기준 상대경로로 지정한다.

```bash
roslaunch trajectory_generator trajectory_publisher.launch \
  trajectory_csv:=traj_sample/200889_200258.csv
```

### 3. 동작 확인

```bash
rostopic hz /control/trajectory
rostopic echo -n 1 /control/trajectory/header
rostopic type /control/debug/generator/path
rostopic type /control/debug/generator/markers
```

RViz에서 다음 topic을 추가하면 생성 경로와 시작/종료 지점을 볼 수 있다.

- `/control/debug/generator/path` (`nav_msgs/Path`)
- `/control/debug/generator/markers` (`visualization_msgs/MarkerArray`)

## 기능과 역할

- `generate_trajectory.py`: drive-course의 successor chain과 Lanelet2 중심선으로 0.1 m 간격
  trajectory CSV를 생성한다.
- `trajectory_publisher_node`: 저장된 CSV를 읽어
  `autoware_planning_msgs/Trajectory`로 30 Hz publish한다.
- generator는 경로를 만들고 publish할 뿐 차량 제어 명령은 만들지 않는다.

주요 입출력은 다음과 같다.

| 구분 | 값 |
| --- | --- |
| 생성 입력 | Lanelet2 OSM, projector YAML, drive-course CSV, global path |
| 생성 출력 | `traj_sample/<start>_<end>.csv` |
| ROS 출력 | `/control/trajectory` |
| 설정 | `config/trajectory_generator.yaml`, `config/trajectory_publisher.yaml` |

세부 생성 규칙과 인터페이스는 [trajectory_gen.md](trajectory_gen.md)를 참고한다.
