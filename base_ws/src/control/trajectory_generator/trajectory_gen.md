# Trajectory Generator 설계 요약

## 목적

`trajectory_generator`는 Lanelet 기반 trajectory를 CSV로 생성하고, 저장된 CSV를
`autoware_planning_msgs/Trajectory`로 publish한다. 생성과 publish는 서로 분리한다.

```text
map + route + anchors -> generate_trajectory.py -> traj_sample/*.csv
traj_sample/*.csv -> trajectory_publisher_node -> /control/trajectory
```

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
| `trajectory.default_velocity_mps` | 속도 제한이 없을 때 사용할 속도 |

Lanelet2 OSM과 drive-course CSV는 같은 map version이어야 한다.

## 생성 규칙

1. drive-course의 `role=preferred_course` 행에서 start 후보를 찾는다.
2. `successor_lanelet_id`를 따라 end까지 도달하는 chain을 계산한다.
3. 유효 chain이 없거나 두 개 이상이면 실패한다.
4. 각 lanelet의 OSM `centerline`을 projector로 로컬 좌표화한다.
5. route endpoint, 이전 중심선, global path를 이용해 중심선 방향을 정한다.
6. 중심선을 연결하고 3차원 누적 거리 기준 0.1 m로 resampling한다.
7. yaw, 목표 속도, lanelet ID, route order를 기록한다.

목표 속도는 drive-course `speed_limit_kph`, OSM speed limit, 기본 속도 순으로 결정한다.
출발/종료 ramp는 만들지 않고 마지막 point의 목표 속도도 강제로 0으로 바꾸지 않는다.
`acceleration_mps2`는 MVP에서 `0.0`이다.

## CSV 계약

파일명과 위치:

```text
traj_sample/<start_lanelet_id>_<end_lanelet_id>.csv
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
