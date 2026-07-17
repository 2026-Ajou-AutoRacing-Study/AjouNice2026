# Trajectory Generator Package

`trajectory_generator`는 ROS1 catkin 패키지이며 기능은 크게 두 가지이다.

1. Lanelet 기반 trajectory 생성 및 파일 저장
2. 저장된 trajectory ROS publish

trajectory 생성과 publish는 분리한다. 생성 기능은 필요할 때 한 번 실행해 파일을 만들고,
publish 기능은 저장된 파일만 읽어 `/control/trajectory`로 반복 publish한다.

## 패키지 구조

```text
base_ws/src/control/trajectory_generator/
  CMakeLists.txt
  package.xml
  trajectory_gen.md

  config/
    trajectory_generator.yaml
    trajectory_publisher.yaml

  launch/
    trajectory_publisher.launch

  scripts/
    generate_trajectory.py

  src/
    trajectory_publisher_node.cpp

  traj_sample/
    <start_lanelet_id>_<end_lanelet_id>.csv
```

| 구성 | 역할 |
| --- | --- |
| `generate_trajectory.py` | 기능 1: trajectory 생성 및 저장 |
| `trajectory_publisher_node.cpp` | 기능 2: 저장 trajectory를 30 Hz로 publish |
| `trajectory_publisher.launch` | publisher YAML 로드, CSV 선택, publisher node 실행 |
| `trajectory_generator.yaml` | 기능 1이 읽는 map, route, anchor, trajectory 설정 |
| `trajectory_publisher.yaml` | 기능 2의 topic, publish rate, frame 설정 |
| `traj_sample/` | 생성된 trajectory CSV 저장 |

`CMakeLists.txt`는 `generate_trajectory.py`를 package executable로 설치하고 C++ publisher를
빌드한다. `package.xml`은 `roscpp`와 필요한 ROS message package 및 runtime dependency를
선언한다. 세부 class나 내부 함수 구성은 이 문서에서 정하지 않는다.

## 공통 설정

### 경로 정책

모든 파일 경로는 저장소 root `AjouNice2026/` 기준 상대경로로 쓴다. 생성 결과 저장
디렉터리는 아래 위치로 고정하며 config에서 변경하지 않는다.

```text
base_ws/src/control/trajectory_generator/traj_sample/
```

ROS topic 이름의 선행 `/`는 파일 경로가 아니다. 아래 실행 예시는 저장소 root에서
수행한다.

### 생성 YAML 예시

`trajectory_generator.yaml`은 기능 1에서만 사용한다. 파일명에 map version을 넣지 않고,
실제로 사용할 map version은 YAML 내부의 map과 drive-course 경로로 결정한다. 따라서
package나 생성 로직은 v44에 종속되지 않는다.

```yaml
map:
  lanelet_map_file: kcity_map/lanelet_map/lanelet2_map_v44.osm
  projector_file: kcity_map/lanelet_map/map_projector_info_v1.yaml
  route_lanelets_file: kcity_map/competition_route/v44/drive_course_v44_lanelets.csv

reference:
  global_path_file: configs/global_path/2026_molit_comp_global_path.txt

anchors:
  start_lanelet_id: 200889
  end_lanelet_id: 200258

trajectory:
  frame_id: map
  resample_interval_m: 0.1
  default_velocity_mps: 3.0
```

Lanelet2 OSM과 drive-course CSV는 같은 map version을 사용해야 한다. 예를 들어 v44 OSM을
사용하면 `competition_route/v44/drive_course_v44_lanelets.csv`를 함께 사용한다.

### Publisher YAML 예시

`trajectory_publisher.yaml`에는 CSV 파일을 제외한 publisher 파라미터를 둔다.

```yaml
trajectory_topic: /control/trajectory
publish_rate_hz: 30.0
frame_id: map
debug_path_topic: /control/debug/generator/path
debug_marker_topic: /control/debug/generator/markers
```

### Projector YAML

Projector YAML은 유지한다.

- OSM node의 `lat/lon/ele`를 Lanelet 로컬 `map` 좌표로 변환한다.
- static TF `map -> morai_map`은 이미 로컬 좌표가 된 두 map frame 사이를 정렬한다.

static TF만으로는 OSM 위경도를 로컬 `(x, y, z)`로 바꿀 수 없으므로 OSM 중심선을 직접
사용하는 trajectory 생성 단계에는 projector 정보가 필요하다.

## 기능 1. Trajectory 생성 및 저장

### 목적

YAML에 지정한 시작 lanelet부터 도착 lanelet까지의 주행 경로를 만들고, follower가 사용할
수 있는 trajectory CSV로 저장한다.

### 입력

| 입력 | 용도 |
| --- | --- |
| Lanelet2 OSM | lanelet별 실제 중심선 좌표 |
| map projector YAML | OSM 위경도를 `map` 좌표로 변환 |
| drive-course lanelet CSV | 주행 순서와 `successor_lanelet_id` 연결 |
| global path TXT | 중심선 방향과 연결 상태 확인 |
| `start_lanelet_id` | 생성 시작 lanelet |
| `end_lanelet_id` | 생성 종료 lanelet |

시작/종료 anchor는 별도 CSV를 사용하지 않고 YAML에 lanelet id 정수로 직접 입력한다.

### 생성 흐름

```text
YAML config
  -> start/end lanelet id 확인
  -> drive-course에서 successor chain 선택
  -> 선택된 lanelet의 OSM centerline 로드
  -> projector를 사용해 map 좌표로 변환
  -> global path 기준 연결 방향 확인
  -> 0.1 m 간격 resampling
  -> yaw와 목표 속도 설정
  -> traj_sample/에 CSV 저장
```

drive-course CSV에서는 lanelet의 `order`, `lanelet_id`, `successor_lanelet_id`를 사용한다.
실제 trajectory 좌표는 CSV의 `centerline_xy`가 아니라 Lanelet2 OSM의 `centerline` way에서
가져온다.

### 중복 lanelet 처리

같은 lanelet id가 drive-course에 여러 번 등장할 수 있다. 따라서 같은 id의 첫 row를
무조건 선택하지 않는다.

- 각 start 후보에서 `successor_lanelet_id`를 따라 end까지 도달 가능한 chain을 찾는다.
- 도달 가능한 chain이 하나이면 해당 chain을 사용한다.
- 도달 가능한 chain이 없으면 생성 실패로 처리한다.
- 두 개 이상이면 start/end id만으로 구간을 확정할 수 없으므로 오류 처리한다.

### Resampling과 속도

- Resample interval은 `0.1 m`로 고정한다.
- 출발 ramp와 종료 ramp는 생성하지 않는다.
- `speed_limit_kph`가 있으면 해당 속도를 사용하고, 없으면 `default_velocity_mps`를 사용한다.
- 첫 point부터 마지막 point까지 유효한 목표 속도를 넣는다.
- 마지막 point의 velocity를 `0.0 m/s`로 강제하지 않는다.
- MVP의 `acceleration_mps2`는 `0.0`으로 둔다.

### 마지막 point 책임

trajectory에는 마지막 point까지 정상 추종할 수 있는 위치, yaw, 목표 속도를 넣는다.
마지막 point 도달 또는 통과 이후의 제어 명령은 generator가 아니라 follower가 처리한다.

```text
accel = 0.0
acceleration = 0.0
front_steer = 0.0
rear_steer = 0.0
```

### 저장 결과

파일명은 시작 lanelet id와 도착 lanelet id로 자동 결정한다.

```text
base_ws/src/control/trajectory_generator/traj_sample/
  <start_lanelet_id>_<end_lanelet_id>.csv
```

예시:

```text
base_ws/src/control/trajectory_generator/traj_sample/200889_200258.csv
```

CSV는 최소한 아래 정보를 저장한다.

```csv
index,x,y,z,yaw,velocity_mps,acceleration_mps2,lanelet_id,route_order
```

별도의 결과 metadata YAML은 만들지 않는다. map, projector, drive-course와 start/end
lanelet id는 입력 `trajectory_generator.yaml`에 이미 있으므로 중복 저장하지 않는다.
같은 start/end 조합이 이미 존재하면 명시적인 overwrite 옵션 없이 덮어쓰지 않는다.

### 실행

```bash
rosrun trajectory_generator generate_trajectory.py \
  --config base_ws/src/control/trajectory_generator/config/trajectory_generator.yaml
```

### 완료 기준

- 지정한 start/end lanelet으로 유일한 successor chain을 결정한다.
- 선택된 lanelet의 중심선을 OSM에서 가져온다.
- trajectory를 `0.1 m` 간격으로 생성한다.
- ramp와 마지막 point 강제 정지 profile을 만들지 않는다.
- `<start_lanelet_id>_<end_lanelet_id>.csv`를 `traj_sample/`에 저장한다.

## 기능 2. 저장 Trajectory Publish

### 목적

기능 1에서 생성한 CSV를 읽어 `autoware_planning_msgs/Trajectory`로 변환하고 ROS topic으로
30 Hz로 반복 publish하는 C++ node이다. publisher는 trajectory geometry를 새로 만들거나
수정하지 않는다.

### 입력 파일 선택

publisher는 generation YAML을 읽지 않는다. publish할 CSV는
`trajectory_publisher.launch`의 `trajectory_csv` argument로 직접 지정한다.

```text
trajectory_csv: traj_sample/200889_200258.csv
```

`trajectory_csv`는 `trajectory_generator` package root 기준 상대경로이다. 파일이 없거나
CSV 형식이 잘못된 경우 publish하지 않고 오류를 출력한다.

### Launch 구성

`trajectory_publisher.launch`는 아래 세 역할만 담당한다.

- `trajectory_publisher.yaml` 로드
- publish할 `trajectory_csv` 선택
- `trajectory_publisher_node.cpp`로 빌드된 node 실행

topic, publish rate, frame id, debug topic은 launch argument로 만들지 않고
`trajectory_publisher.yaml`에서 읽는다. trajectory 생성 script, generation YAML 로드,
follower, validator, RViz 실행은 포함하지 않는다.

```xml
<launch>
  <arg name="trajectory_csv" default="traj_sample/200889_200258.csv"/>

  <node pkg="trajectory_generator"
        type="trajectory_publisher_node"
        name="trajectory_publisher"
        output="screen">
    <rosparam command="load"
              file="$(find trajectory_generator)/config/trajectory_publisher.yaml"/>
    <param name="trajectory_csv" value="$(arg trajectory_csv)"/>
  </node>
</launch>
```

### Publish topic

| Topic | Type | 용도 |
| --- | --- | --- |
| `/control/trajectory` | `autoware_planning_msgs/Trajectory` | follower 입력 |
| `/control/debug/generator/path` | `nav_msgs/Path` | RViz trajectory 확인 |
| `/control/debug/generator/markers` | `visualization_msgs/MarkerArray` | start/end와 point 확인 |

Publish 정책:

- 저장 CSV를 시작할 때 한 번 읽는다.
- `trajectory_publisher.yaml`의 `publish_rate_hz: 30.0`에 맞춰 trajectory와 debug topic을 모두 30 Hz로 publish한다.
- `header.frame_id`는 `trajectory_publisher.yaml`의 `frame_id`를 사용한다.
- `header.stamp`는 publish 시각으로 갱신한다.
- MVP에서는 hot reload를 지원하지 않는다.
- trajectory를 변경하려면 기능 1을 다시 실행한 뒤 publisher를 재시작한다.

### 실행

```bash
roslaunch trajectory_generator trajectory_publisher.launch \
  trajectory_csv:=traj_sample/200889_200258.csv
```

### 확인

```bash
rostopic hz /control/trajectory
rostopic echo /control/trajectory/header
```

RViz에서는 아래 topic을 확인한다.

- `/control/debug/generator/path`
- `/control/debug/generator/markers`

### 완료 기준

- launch의 `trajectory_csv`로 지정한 CSV를 `traj_sample/`에서 읽는다.
- 저장된 모든 point를 순서대로 `autoware_planning_msgs/Trajectory`에 넣는다.
- C++ publisher가 `/control/trajectory`와 RViz debug topic을 모두 30 Hz로 publish한다.
- publisher가 저장 trajectory의 geometry와 velocity를 변경하지 않는다.

## 패키지 책임 경계

| 패키지 | 책임 |
| --- | --- |
| `trajectory_generator` | trajectory 생성, 저장, publish |
| `trajectory_follower` | 마지막 point까지 추종하고 이후 accel/steering을 0으로 처리 |
| `trajectory_validator` | 기준 trajectory와 실제 주행 결과 기록 및 평가 |
