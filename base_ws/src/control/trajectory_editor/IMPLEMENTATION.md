# Trajectory Editor Implementation

`trajectory_editor`는 제어 파이프라인에서 trajectory source 역할만 담당한다.
제어 명령을 만들거나 MORAI에 `/ctrl_cmd`를 publish하지 않는다. 우선 구현 기능은
아래 두 가지이다.

1. YAML config에 적힌 map/route/start/end 입력을 기준으로 실제 주행 trajectory를 만든다.
2. 만든 trajectory를 YAML config에 적힌 경로에 저장한다.
3. launch로 실행될 때 저장/생성된 trajectory를 `/control/trajectory`로 publish한다.

## 기본 아이디어

사용자는 생성용 YAML config를 준다. YAML 안에는 Lanelet2 map, 같은 버전의 주행
course lanelet CSV, 시작/종료 lanelet id CSV, 저장 경로가 들어간다. 따라서
`lanelet2_map_v44.osm`에서 `lanelet2_map_v45.osm`처럼 map version이 바뀌어도 YAML의
파일 경로만 바꿔서 같은 Python 생성 로직을 실행할 수 있어야 한다.

최종 전역경로는 일반 실행 중 자주 바뀌는 값이 아니라, 대회/미션 기준으로 고정된
reference path로 본다. 기본값은 `configs/global_path/2026_molit_comp_global_path.txt`를
사용한다. 사용자가 매번 바꾸는 입력은 주로 시작 CSV와 종료 CSV이고, 이 CSV는 좌표가
아니라 `lanelet_id`를 담는다.

- Lanelet2 OSM map: 예) `kcity_map/lanelet_map/lanelet2_map_v44.osm`, `lanelet2_map_v45.osm`
- 같은 버전의 주행 course lanelet CSV: 예) `kcity_map/competition_route/v44/drive_course_v44_lanelets.csv`
- map projector YAML: 예) `kcity_map/lanelet_map/map_projector_info_v1.yaml`
- 시작 CSV: `lanelet_id` 입력
- 종료 CSV: `lanelet_id` 입력
- 최종 전역경로: 예) `configs/global_path/2026_molit_comp_global_path.txt` 고정 기준선
- 저장 경로: 생성된 trajectory CSV 또는 YAML 저장 위치

`trajectory_editor`는 시작 lanelet id부터 종료 lanelet id까지 자연스럽게 이어지는
trajectory를 생성한다. 생성되는 trajectory는 기본적으로 `drive_course_*_lanelets.csv`에
정리된 lanelet 순서와 중심선 좌표를 따른다. 단, 중심선은 최종 전역경로를 기준으로
검사하고 필요한 경우 보정한다.

즉 생성 기준의 우선순위는 아래와 같다.

1. YAML config를 읽는다.
2. Lanelet2 OSM과 같은 버전의 drive-course lanelet CSV를 로드한다.
3. start/end CSV의 `lanelet_id`를 drive-course lanelet 순서표에서 찾는다.
4. start lanelet부터 end lanelet까지의 centerline을 이어 붙인다.
5. 최종 전역경로로 전체 진행 방향과 station 구간을 검사/보정한다.
6. follower가 바로 추종할 수 있도록 일정 간격으로 resampling하고 velocity를 채운다.
7. 생성된 trajectory를 YAML config의 output path에 저장한다.

## 전체 동작

```text
trajectory_config.yaml
  -> scripts/generate_trajectory.py
  -> trajectory 생성
  -> output_trajectory_file 저장

roslaunch trajectory_editor trajectory_publisher.launch
  -> scripts/trajectory_publisher_node.py
  -> trajectory_config.yaml 로드
  -> output_trajectory_file 로드
  -> publish loop

Lanelet2 OSM + projector
  -> lanelet centerline 후보 추출

drive_course_*_lanelets.csv
  -> version-matched route lanelet order
  -> lanelet_id별 centerline_xy

global path txt
  -> 최종 전역경로 polyline
  -> station index 생성

start CSV + end CSV
  -> lanelet_id 로드
  -> route order에서 start/end row 결정

route lanelet rows + global path reference
  -> 중심선 연결
  -> global path 기준 보정
  -> resampling / yaw / velocity profile
  -> autoware_planning_msgs/Trajectory
  -> output_trajectory_file 저장
  -> /control/trajectory
```

MVP 생성 구현은 `generate_trajectory.py` 하나의 Python 파일로 구성한다. 이 파일은
ROS launch 없이 직접 실행되며, trajectory를 생성하고 저장하는 것까지만 담당한다.
파일 내부에서 loader, route slicing, centerline 연결, 보정, resampling, message/export
데이터 생성을 함수 단위로 나눈다.

publish는 별도 launch로 실행한다. publish launch는 저장된 trajectory를 읽어 ROS topic으로
쏘는 역할만 담당하고, trajectory geometry를 새로 만들지 않는다.

## 입력 파일

### 설정 YAML

생성 Python은 개별 파일 경로를 코드에 박지 않고 YAML config 하나를 입력으로 받는다.
YAML에는 map, route lanelet CSV, start/end CSV, 고정 global path, 저장 경로가 들어간다.

예시:

```yaml
map:
  lanelet_map_file: C:/Users/wpql0/OneDrive/Desktop/project/AjouNice2026/kcity_map/lanelet_map/lanelet2_map_v44.osm
  projector_file: C:/Users/wpql0/OneDrive/Desktop/project/AjouNice2026/kcity_map/lanelet_map/map_projector_info_v1.yaml
  route_lanelets_file: C:/Users/wpql0/OneDrive/Desktop/project/AjouNice2026/kcity_map/competition_route/v44/drive_course_v44_lanelets.csv

reference:
  global_path_file: C:/Users/wpql0/OneDrive/Desktop/project/AjouNice2026/configs/global_path/2026_molit_comp_global_path.txt

anchors:
  start_csv: C:/path/to/start.csv
  end_csv: C:/path/to/end.csv

output:
  trajectory_file: C:/path/to/generated/trajectory.csv
  metadata_file: C:/path/to/generated/trajectory_meta.yaml

trajectory:
  frame_id: map
  resample_interval_m: 0.5
  default_velocity_mps: 3.0
  start_ramp_length_m: 5.0
  end_ramp_length_m: 8.0
  stop_last_point: true
```

YAML의 `lanelet_map_file`과 `route_lanelets_file`은 같은 map version을 기준으로 맞춘다.
예를 들어 `lanelet2_map_v44.osm`을 쓰면 `competition_route/v44/drive_course_v44_lanelets.csv`를
같이 써야 한다. `lanelet2_map_v45.osm`으로 바꾸면 route lanelet CSV도 v45 기준 파일로
바꾼다.

### 파일 경로 인자 정책

`trajectory_editor`는 특정 map version에 종속되면 안 된다.

필수 입력 파일은 모두 아래처럼 외부 인자로 받는다.

| 입력 | ROS param | launch arg | 비고 |
| --- | --- | --- | --- |
| Config YAML | `~config_file` | `config_file` | 생성 설정의 단일 진입점 |
| Lanelet2 OSM | `~lanelet_map_file` | `lanelet_map_file` | v44/v45 등 버전 변경 가능 |
| Drive-course lanelets CSV | `~route_lanelets_file` | `route_lanelets_file` | OSM과 같은 version |
| Projector YAML | `~projector_file` | `projector_file` | map 좌표 변환 기준 |
| Global path TXT | `~global_path_file` | `global_path_file` | 최종 전역경로, 기본 고정 reference |
| Start CSV | `~start_csv` | `start_csv` | 시작 `lanelet_id` |
| End CSV | `~end_csv` | `end_csv` | 종료 `lanelet_id` |
| Output trajectory | `~output_trajectory_file` | `output_trajectory_file` | 생성 결과 저장 경로 |

금지할 구현:

- 코드 안에 `lanelet2_map_v44.osm` 같은 특정 파일명을 박아두는 것
- package 내부 상대 경로만 가정하는 것
- map version별로 node 코드를 나누는 것

허용할 구현:

- launch default로 현재 기본 map인 `lanelet2_map_v44.osm`를 지정
- 실행 시 `lanelet_map_file:=.../lanelet2_map_v45.osm`로 override
- config YAML에서 기본 파일 경로를 관리하고 launch arg로 교체
- generation script는 `config_file` 하나만 받아도 동작 가능하게 구성

### Lanelet2 OSM

Lanelet2 OSM은 도로 중심선과 차선 경계 정보를 제공한다.

초기 구현에서는 relation 중 아래 조건을 만족하는 lanelet을 사용한다.

- `type=lanelet`
- `subtype=road`
- `centerline` role을 가진 way가 있음

각 OSM node는 `lat`, `lon`, `ele`를 가진다. `map_projector_info_v1.yaml`의
`LocalCartesianUTM` 설정을 사용해 로컬 `map` frame의 `(x, y, z)`로 변환한다.

기존 `scripts/map/kcity/kcity_map_align.py`에는 Lanelet2 OSM을 파싱하고
`LocalCartesianUTM`으로 변환하는 함수가 있으므로, 구현 시 해당 로직을 재사용하거나
동일한 방식으로 분리한다.

### Drive-course lanelet CSV

`drive_course_v44_lanelets.csv`는 해당 map version에서 대회 주행 경로를 이루는 lanelet
순서표이다. 이 파일은 start/end lanelet id를 route 구간으로 바꾸는 핵심 입력이다.

주요 column:

| Column | 용도 |
| --- | --- |
| `order` | 주행 순서 |
| `lanelet_id` | 구간 선택 기준 id |
| `successor_lanelet_id` | 다음 lanelet 연결 확인 |
| `speed_limit_kph` | 기본 속도 profile 후보 |
| `length_m` | 구간 길이 |
| `centerline_xy` | `x:y:z;...` 형태의 중심선 좌표 |

MVP에서는 OSM에서 centerline을 다시 routing하기보다, 같은 version의
`drive_course_*_lanelets.csv`에 이미 정리된 `order`와 `centerline_xy`를 우선 사용한다.
Lanelet2 OSM은 version validation, map frame 기준, 필요 시 centerline 재확인용으로 둔다.

### 시작 CSV / 종료 CSV

시작 CSV와 종료 CSV는 trajectory를 만들 구간의 시작 lanelet id와 종료 lanelet id를 지정한다.

MVP에서는 각 CSV가 최소 하나의 `lanelet_id`를 가진다고 가정한다.

권장 형식:

```csv
lanelet_id
200889
```

여러 point가 들어있는 경우:

- 시작 CSV는 첫 번째 `lanelet_id`를 start anchor로 사용한다.
- 종료 CSV는 첫 번째 `lanelet_id`를 end anchor로 사용한다.
- 실제 구현에서는 `~start_anchor_mode`, `~end_anchor_mode`로 선택 가능하게 둔다.

확장 형식:

```csv
lanelet_id,offset_m
200889,0.0
```

`offset_m`은 해당 lanelet centerline 안에서 시작/종료를 더 세밀하게 자를 때 사용한다.
초기 MVP에서는 `offset_m` 없이 lanelet 단위로 자른다. 좌표 기반 start/end 입력은 확장 기능으로 둔다.

### 최종 전역경로 TXT

전역경로는 줄마다 공백으로 구분된 `x y z` point list이다. 이 파일은 대회 전체
주행 기준이 되는 고정 reference로 취급한다.

예시:

```text
-131.68979755061446 -428.3310229377821 28.543960281954277
-131.44967144399345 -427.8924578823263 28.544341139013788
-131.20954533737245 -427.4538928268705 28.5447219960733
```

전역경로의 역할:

- drive-course lanelet 구간이 최종 전역경로 방향과 맞는지 검사한다.
- route lanelet centerline이 최종 경로와 가까운지 확인하는 기준이 된다.
- Lanelet centerline이 끊기거나 흔들리는 구간을 보정하는 기준이 된다.
- 생성된 trajectory의 station, yaw, smoothing 품질을 검증하는 reference가 된다.

운영 정책:

- node 시작 시 한 번 로드하고 station index를 만든다.
- 일반 MVP에서는 runtime hot reload를 지원하지 않는다.
- 시작/종료 lanelet id가 바뀌어도 같은 global path를 기준으로 검사/보정한다.
- global path 자체를 바꿔야 하는 경우에만 `global_path_file` launch argument를 override한다.

## 기능 1. Trajectory 생성

### 1. 입력 로드

로드할 데이터:

- config YAML
- lanelet OSM
- projector YAML
- drive-course lanelet CSV
- global path TXT, 기본적으로 고정 reference라 node 시작 시 1회 로드
- start CSV
- end CSV

출력 중간 데이터:

- `route_lanelets`: `order`, `lanelet_id`, `successor_lanelet_id`, `centerline_xy` row list
- `global_path_points`: 전역경로 `(x, y, z)` list
- `global_path_station`: 누적 거리 `s` list
- `start_lanelet_id`, `end_lanelet_id`: 시작/종료 기준 lanelet id
- `selected_route_rows`: start부터 end까지 잘라낸 drive-course lanelet rows

### 2. 시작/종료 lanelet 결정

start/end CSV에서 `lanelet_id`를 읽고 drive-course lanelet CSV에서 해당 row를 찾는다.

```text
start.csv -> start_lanelet_id -> route order start_order
end.csv   -> end_lanelet_id   -> route order end_order
```

기본 정책:

- start/end lanelet id가 route CSV에 없으면 생성 실패로 처리한다.
- `start_order <= end_order`이면 정방향 구간으로 trajectory를 만든다.
- `start_order > end_order`이면 기본적으로 오류 처리한다.
- reverse 또는 loop 구간이 필요하면 별도 정책을 추가한다.
- `successor_lanelet_id`가 다음 row의 `lanelet_id`와 맞지 않는 구간은 warning 또는 오류 처리한다.

필요 파라미터:

| Param | 기본값 후보 | 설명 |
| --- | ---: | --- |
| `~allow_reverse` | `false` | end order가 start order보다 작을 때 reverse 생성 허용 |
| `~strict_successor_check` | `true` | route CSV의 successor 연결 검사 |

### 3. Drive-course 구간 추출

`start_order`부터 `end_order`까지 drive-course lanelet rows를 자른다.

```text
selected_route_rows = route_lanelets[start_order:end_order]
```

이 구간의 `centerline_xy`를 순서대로 이어 붙인 것이 trajectory의 1차 중심선이다.
각 row의 `speed_limit_kph`, `maneuver`, `length_m`은 velocity profile 또는 debug metadata에
사용할 수 있다.

### 4. Lanelet 중심선 연결

MVP에서는 `drive_course_*_lanelets.csv`의 `centerline_xy`를 우선 사용한다.
`centerline_xy`는 아래처럼 `;`로 point를 나누고, 각 point는 `x:y:z`이다.

```text
x1:y1:z1;x2:y2:z2;x3:y3:z3
```

연결 규칙:

- selected route rows를 `order` 순서로 정렬한다.
- 각 row의 `centerline_xy`를 point list로 파싱한다.
- 이전 row 마지막 point와 다음 row 첫 point가 중복되거나 매우 가까우면 하나만 유지한다.
- 연결 거리가 너무 크면 warning 또는 오류 처리한다.
- lanelet id와 OSM relation id가 일치하는지 필요 시 검증한다.

확장 구현에서는 drive-course CSV가 없거나 깨진 경우 Lanelet2 relation adjacency를 이용해
lanelet chain을 직접 찾는다.

### 5. Global path 기반 중심선 보정

trajectory는 drive-course lanelet의 중심선 기준으로 만들지만, 최종 전역경로와 크게
어긋나는 중심선은 그대로 사용하지 않는다.

보정 원칙:

- route centerline point를 global path에 projection한다.
- projection distance가 작으면 centerline point를 유지한다.
- projection distance가 크면 global path point 쪽으로 보정한다.
- centerline이 비어 있거나 깨진 구간은 global path를 fallback으로 사용한다.

간단한 보정식:

```text
corrected_point = centerline_point * (1 - alpha) + global_path_projection * alpha
```

`alpha`는 centerline과 global path 사이 거리에 따라 정한다.

| 상황 | alpha |
| --- | ---: |
| 거리 작음 | `0.0` |
| 거리 중간 | `0.0 ~ 0.5` |
| 거리 큼 | `0.5 ~ 1.0` |
| centerline 없음 | `1.0` |

MVP에서는 더 단순하게 `max_centerline_error_m`를 넘는 점만 global path projection으로
대체해도 된다.

### 6. Resampling

보정된 polyline을 follower가 안정적으로 쓸 수 있도록 일정 간격으로 다시 샘플링한다.

권장:

- 기본 point 간격: `0.5 m`
- 너무 짧은 segment는 제거
- 중복 point 제거
- yaw는 인접 point 방향으로 계산
- 마지막 point yaw는 직전 yaw를 사용

### 7. Velocity profile 생성

초기 MVP는 단순 velocity profile로 시작한다.

- 기본 주행 속도: `~default_velocity_mps`
- route CSV의 `speed_limit_kph`가 있으면 `default_velocity_mps`보다 우선할 수 있다.
- 시작 구간: 0에서 기본 속도까지 ramp-up
- 종료 구간: 기본 속도에서 0까지 ramp-down
- 마지막 point: `0.0 m/s`

추후에는 곡률 기반 감속, 구간별 속도 CSV, stop point 설정을 추가한다.

### 8. Trajectory 메시지 생성

생성 결과는 `autoware_planning_msgs/Trajectory`이다.

```text
std_msgs/Header header
TrajectoryPoint[] points
```

각 `TrajectoryPoint`는 MVP에서 아래처럼 채운다.

| Field | 값 |
| --- | --- |
| `header.frame_id` | `map` |
| `header.stamp` | publish 시각 |
| `pose.position.x/y/z` | resampled point |
| `pose.orientation` | yaw를 quaternion으로 변환 |
| `longitudinal_velocity_mps` | velocity profile |
| `lateral_velocity_mps` | `0.0` |
| `acceleration_mps2` | `0.0` 또는 profile 기반 추정 |
| `heading_rate_rps` | `0.0` |
| `front_wheel_angle_rad` | `0.0` |
| `rear_wheel_angle_rad` | `0.0` |

### 9. Trajectory 저장

생성 Python은 최종 trajectory를 YAML config의 `output.trajectory_file` 경로에 저장한다.
MVP 저장 형식은 사람이 확인하기 쉬운 CSV로 둔다.

권장 저장 column:

```csv
index,x,y,z,yaw,velocity_mps,acceleration_mps2,lanelet_id,route_order
0,-0.131,117.580,28.544,1.062,0.0,0.0,200889,1
1,1.155,119.929,28.546,1.060,3.0,0.0,200889,1
```

저장 파일의 역할:

- 생성 결과를 재현 가능하게 남긴다.
- publish launch가 동일 trajectory를 다시 읽어 publish할 수 있게 한다.
- validator가 기준 trajectory로 사용할 수 있게 한다.
- tuning 중 trajectory geometry가 바뀌었는지 diff로 확인할 수 있게 한다.

`output.metadata_file`에는 사용한 config, map version, start/end lanelet id, point 수,
총 길이, 생성 시간 등을 저장한다.

## 기능 2. Trajectory publish

### Publish topic

| Topic | Type | 비고 |
| --- | --- | --- |
| `/control/trajectory` | `autoware_planning_msgs/Trajectory` | follower 기본 입력 |
| `/control/debug/editor/path` | `nav_msgs/Path` | RViz 확인용 |
| `/control/debug/editor/markers` | `visualization_msgs/MarkerArray` | centerline/global path/debug point 확인 |

### Publish 정책

- publish는 `trajectory_publisher.launch`로 실행한다.
- launch는 `config_file`을 받아 `trajectory_publisher_node.py`를 ROS node로 실행한다.
- publisher node는 config의 `output.trajectory_file`을 읽어 `/control/trajectory`로 publish한다.
- publisher node는 trajectory를 새로 생성하지 않는다.
- trajectory geometry를 바꾸려면 먼저 `generate_trajectory.py`를 다시 실행해 저장 파일을 갱신한다.
- `publish_rate_hz`에 맞춰 반복 publish한다.
- `header.stamp`는 매 publish마다 현재 ROS time으로 갱신한다.
- 저장된 trajectory 파일이 바뀌지 않으면 trajectory geometry는 유지한다.
- MVP에서는 hot reload를 지원하지 않는다.
- debug path는 최종 trajectory 기준으로 publish한다.
- debug marker는 start/end lanelet, selected drive-course centerline, corrected path를 구분해서 보여준다.

## ROS parameters

| Param | 기본값 후보 | 설명 |
| --- | --- | --- |
| `~config_file` | `""` | 생성 YAML config |
| `~lanelet_map_file` | `$(find ...)/kcity_map/lanelet_map/lanelet2_map_v44.osm` | Lanelet2 OSM |
| `~route_lanelets_file` | `$(find ...)/kcity_map/competition_route/v44/drive_course_v44_lanelets.csv` | 같은 version의 drive-course lanelet CSV |
| `~projector_file` | `$(find ...)/kcity_map/lanelet_map/map_projector_info_v1.yaml` | LocalCartesianUTM projector |
| `~global_path_file` | `$(find ...)/configs/global_path/2026_molit_comp_global_path.txt` | 최종 전역경로, 기본 고정 reference |
| `~start_csv` | `""` | 시작 lanelet id CSV |
| `~end_csv` | `""` | 종료 lanelet id CSV |
| `~output_trajectory_file` | `""` | 생성 trajectory CSV 저장 경로 |
| `~output_metadata_file` | `""` | 생성 metadata YAML 저장 경로 |
| `~trajectory_topic` | `/control/trajectory` | trajectory publish topic |
| `~debug_path_topic` | `/control/debug/editor/path` | debug path topic |
| `~debug_marker_topic` | `/control/debug/editor/markers` | debug marker topic |
| `~frame_id` | `map` | trajectory frame |
| `~publish_rate_hz` | `10.0` | publish 주기 |
| `~resample_interval_m` | `0.5` | trajectory point 간격 |
| `~default_velocity_mps` | `3.0` | 기본 목표 속도 |
| `~start_ramp_length_m` | `5.0` | 출발 가속 구간 |
| `~end_ramp_length_m` | `8.0` | 종료 감속 구간 |
| `~stop_last_point` | `true` | 마지막 point velocity를 0으로 강제 |
| `~max_centerline_error_m` | `2.0` | 초과 시 global path로 보정 |
| `~allow_reverse` | `false` | reverse 구간 허용 여부 |
| `~strict_successor_check` | `true` | route CSV successor 연결 검사 |

## 구현 파일 구성

```text
trajectory_editor/
  config/
    trajectory_editor_v44.yaml
  launch/
    trajectory_publisher.launch
  scripts/
    generate_trajectory.py
    trajectory_publisher_node.py
```

| 파일 | 책임 |
| --- | --- |
| `trajectory_editor_v44.yaml` | map/route/start/end/output 경로와 생성 파라미터 |
| `generate_trajectory.py` | YAML config를 읽어 trajectory 생성 및 파일 저장 |
| `trajectory_publisher.launch` | 저장된 trajectory를 publish하는 ROS node 실행 |
| `trajectory_publisher_node.py` | 저장된 trajectory CSV를 읽어 `/control/trajectory` publish |

`generate_trajectory.py` 내부 함수 구성 후보:

| 함수 | 책임 |
| --- | --- |
| `load_config()` | YAML config 로드 및 ROS param override 병합 |
| `load_projector()` | projector YAML 로드 |
| `load_lanelet_centerlines()` | Lanelet2 OSM 로드 및 centerline 추출 |
| `load_route_lanelets()` | drive-course lanelet CSV 로드 및 `centerline_xy` 파싱 |
| `load_global_path()` | 고정 전역경로 TXT 로드 |
| `load_lanelet_anchor_csv()` | start/end CSV에서 `lanelet_id` 로드 |
| `slice_route_by_lanelet_id()` | start/end lanelet id로 route rows 추출 |
| `build_route_centerline()` | selected route rows의 `centerline_xy` 연결 |
| `correct_path_with_global_reference()` | centerline을 global path 기준으로 보정 |
| `resample_path()` | 일정 간격 point 생성 |
| `save_trajectory_file()` | 생성 trajectory CSV와 metadata YAML 저장 |
| `write_metadata_file()` | 생성 조건과 결과 summary 저장 |

MVP에서는 위 함수들을 하나의 `generate_trajectory.py` 안에 둔다. 여러 파일로 나누는 것은 초기 구현 범위가
아니다.

`trajectory_publisher_node.py` 내부 함수 구성 후보:

| 함수 | 책임 |
| --- | --- |
| `load_config()` | YAML config 로드 |
| `load_saved_trajectory()` | `output.trajectory_file` CSV 로드 |
| `build_trajectory_msg()` | 저장된 row를 `autoware_planning_msgs/Trajectory`로 변환 |
| `build_debug_path()` | RViz 확인용 `nav_msgs/Path` 생성 |
| `publish_loop()` | `/control/trajectory`, debug topic 반복 publish |

## 실행 방식

### 생성 실행

trajectory 생성은 launch가 아니라 Python script를 직접 실행한다.

```bash
python scripts/generate_trajectory.py --config /path/to/trajectory_editor_v44.yaml
```

이 명령은 YAML의 `output.trajectory_file`과 `output.metadata_file`을 생성한다.

### Publish launch 초안

trajectory publish는 launch로 실행한다. launch는 기본적으로 `config_file` 하나를
argument로 받고, `trajectory_publisher_node.py`를 ROS node로 실행한다. 세부 파일 경로는
YAML 안에서 관리한다.

```xml
<launch>
  <arg name="config_file" default="$(find trajectory_editor)/config/trajectory_editor_v44.yaml"/>

  <node pkg="trajectory_editor"
        type="trajectory_publisher_node.py"
        name="trajectory_publisher"
        output="screen">
    <param name="config_file" value="$(arg config_file)"/>
    <param name="trajectory_topic" value="/control/trajectory"/>
    <param name="debug_path_topic" value="/control/debug/editor/path"/>
    <param name="debug_marker_topic" value="/control/debug/editor/markers"/>
    <param name="publish_rate_hz" value="10.0"/>
  </node>
</launch>
```

map version을 바꿀 때는 node 코드를 수정하지 않고 config YAML만 바꾼다.

```bash
python scripts/generate_trajectory.py --config /path/to/trajectory_editor_v45.yaml

roslaunch trajectory_editor trajectory_publisher.launch \
  config_file:=/path/to/trajectory_editor_v45.yaml
```

## 검증 방법

```bash
python scripts/generate_trajectory.py --config /path/to/trajectory_editor_v44.yaml

roslaunch trajectory_editor trajectory_publisher.launch \
  config_file:=/path/to/trajectory_editor_v44.yaml

rostopic echo /control/trajectory/header
rostopic hz /control/trajectory
rostopic echo /control/trajectory/points[0]
```

RViz 확인:

- `/control/debug/editor/path`: 최종 publish trajectory
- `/control/debug/editor/markers`: start/end lanelet, selected drive-course centerline, corrected path

## 완료 기준

- [ ] trajectory 생성 로직은 MVP 기준 `generate_trajectory.py` 하나에 들어간다.
- [ ] 생성 입력은 YAML config 하나로 관리한다.
- [ ] trajectory 생성은 launch 없이 Python script 직접 실행으로 수행한다.
- [ ] 생성 결과는 YAML config의 output 경로에 저장된다.
- [ ] trajectory publish는 `trajectory_publisher.launch`로 저장 파일을 읽어 publish하는 방식이다.
- [ ] Lanelet2 OSM과 projector YAML을 읽어 centerline 후보를 만든다.
- [ ] `lanelet_map_file` 인자만 바꿔도 v44/v45 등 다른 OSM map을 로드할 수 있다.
- [ ] 같은 version의 `drive_course_*_lanelets.csv`를 읽어 route lanelet 순서를 만든다.
- [ ] 고정 최종 전역경로 TXT를 node 시작 시 읽어 station index를 만든다.
- [ ] 시작 CSV와 종료 CSV의 `lanelet_id`를 route CSV에서 찾아 start/end order를 구한다.
- [ ] start order부터 end order까지 drive-course centerline을 연결한다.
- [ ] centerline을 최종 전역경로 기준으로 보정한다.
- [ ] 보정된 path를 일정 간격으로 resampling한다.
- [ ] yaw와 velocity profile을 채워 `autoware_planning_msgs/Trajectory`를 만든다.
- [ ] 생성 trajectory CSV와 metadata YAML을 config의 output 경로에 저장한다.
- [ ] `/control/trajectory`가 설정한 주기로 publish된다.
- [ ] RViz에서 최종 trajectory가 도로 중심선을 따라 자연스럽게 보인다.
- [ ] follower가 같은 topic을 subscribe해 주행 입력으로 사용할 수 있다.
