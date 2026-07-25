# Trajectory Generator 설계 요약

이 문서는 유지보수자를 위한 개념 및 interface 계약이다. 처음 실행하는 사용자는
[README.md](README.md)부터 읽는다.

## 목적

`trajectory_generator`는 Lanelet 기반 trajectory를 CSV로 생성하고, 저장된 CSV를
`autoware_planning_msgs/Trajectory`로 publish한다. 생성과 publish는 서로 분리한다.

```text
map + route + anchors -> generate_trajectory.py -> traj_sample/*.csv
traj_sample/*.csv -> trajectory_publisher_node -> /control/trajectory
```

## 핵심 개념

### Path와 trajectory

이 패키지에서 path는 주로 공간상의 선형상 `(x, y, z, yaw)`을 뜻한다. trajectory는 여기에
각 점의 목표 속도와 가속도, 시간 정보가 더해진 제어기 입력이다. 따라서 같은 공간 경로라도
`desired_velocity_kph`가 다르면 서로 다른 trajectory가 된다.

### Lanelet, route, anchor

- **Lanelet**은 도로의 작은 주행 가능 구간이며 OSM relation ID로 식별한다.
- **Route**는 drive-course CSV의 `successor_lanelet_id`로 이어지는 lanelet 순서다.
- **Anchor**는 전체 route 중 실제 CSV로 잘라낼 시작과 종료 lanelet ID다.

generator는 임의의 두 점 사이를 탐색하지 않는다. `role=preferred_course`인 start 행에서
successor를 따라 end까지 도달하는 chain이 정확히 하나일 때만 생성한다. 이는 잘못된 분기 경로를
조용히 선택하는 것을 막기 위한 조건이다.

### 좌표계와 진행 방향

OSM 중심선의 위경도는 projector YAML을 이용해 로컬 `map` 좌표로 변환한다. OSM centerline의
노드 저장 순서가 실제 주행 방향과 같다는 보장이 없으므로 drive-course의 start/end 좌표, 직전
lanelet 끝점, global path의 진행 순서를 함께 비교해 방향을 결정한다. 모든 입력은 동일한 지도와
좌표 기준에서 만들어져야 한다.

### 생성과 발행의 분리

Python generator는 설정이 바뀔 때 CSV를 한 번 만드는 오프라인 도구다. C++ publisher는 시작
시 CSV를 한 번 검증·로딩하고 같은 geometry를 주기적으로 발행하는 온라인 ROS node다. 이 분리로
주행 중 지도 처리 비용을 없애고, 실제 제어에 사용한 입력 CSV를 재현 가능한 결과물로 남긴다.

## 생성 입력

모든 경로는 `AjouNice2026/` 기준 상대경로다.

| 설정 | 의미 |
| --- | --- |
| `map.lanelet_map_file` | 중심선을 읽을 Lanelet2 OSM |
| `map.projector_file` | 위경도를 `map` 좌표로 변환할 projector |
| `map.route_lanelets_file` | 주행 순서와 successor를 가진 drive-course CSV |
| `reference.global_path_file` | 중심선 방향 확인용 기준 경로 |
| `anchors.start_lanelet_id` | 시작 lanelet |
| `anchors.end_lanelet_id` | 종료 lanelet |
| `trajectory.resample_interval_m` | 고정값 `0.1` m |
| `trajectory.desired_velocity_kph` | 사용자가 지정하는 목표 주행 속도 |
| `trajectory.default_speed_limit_kph` | 지도에 제한속도가 없을 때 사용할 안전 상한 |

Lanelet2 OSM과 drive-course CSV는 같은 map version이어야 한다.

## 생성 규칙

1. drive-course의 `role=preferred_course` 행에서 start 후보를 찾는다.
2. `successor_lanelet_id`를 따라 end까지 도달하는 chain을 계산한다.
3. 유효 chain이 없거나 두 개 이상이면 실패한다.
4. 각 lanelet의 OSM `centerline`을 projector로 로컬 좌표화한다.
5. route endpoint, 이전 중심선, global path를 이용해 중심선 방향을 정한다.
6. 중심선을 연결하고 3차원 누적 거리 기준 0.1 m로 resampling한다.
7. yaw, 목표 속도, lanelet ID, route order를 기록한다.

제한속도는 drive-course `speed_limit_kph`, OSM speed limit,
`default_speed_limit_kph` 순으로 결정한다. 이 값은 목표 속도가 아니라 상한이다. 각 점의 최종
목표 속도는 `min(desired_velocity_kph, speed_limit_kph) / 3.6`으로 계산해 m/s로 기록한다.
출발/종료 ramp는 만들지 않고 마지막 point의 목표 속도도 강제로 0으로 바꾸지 않는다.
`acceleration_mps2`는 MVP에서 `0.0`이다.

## CSV 계약

파일명과 위치:

```text
traj_sample/<start_lanelet_id>_<end_lanelet_id>_<desired_velocity>kph.csv
```

필수 column:

```csv
index,x,y,z,yaw,velocity_mps,acceleration_mps2,lanelet_id,route_order
```

`index`는 0부터 연속이어야 하며 모든 수치는 finite여야 한다. 기존 파일은 `--overwrite`
없이는 덮어쓰지 않고, 저장은 임시 파일을 거쳐 원자적으로 교체한다.

## Publish 계약

`trajectory_publisher_node`는 시작할 때 CSV를 한 번 읽고 geometry를 변경하지 않는다.

| Topic | Type | 기본 주기 |
| --- | --- | --- |
| `/control/trajectory` | `autoware_planning_msgs/Trajectory` | 30 Hz |
| `/control/debug/generator/path` | `nav_msgs/Path` | 30 Hz |
| `/control/debug/generator/markers` | `visualization_msgs/MarkerArray` | 30 Hz |

`header.frame_id`는 publisher config를 따르고 `header.stamp`는 매 publish 시 갱신한다.
CSV 누락, 잘못된 column/index/value, 음수 목표 속도는 node 시작 실패로 처리한다.

실행 방법은 [README.md](README.md)를 참고한다.
