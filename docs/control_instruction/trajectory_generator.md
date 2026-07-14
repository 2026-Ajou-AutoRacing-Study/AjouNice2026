# `trajectory_generator` 설계 지시서

## 1. 목표와 우선순위

`trajectory_generator`는 제어기가 따라갈 기준 궤적을 쉽게 만들고 재현 가능하게 publish하는 Python 패키지다.

이 문서는 Lanelet CSV부터 `Trajectory`와 speed profile을 만드는 상세 기준이다. package 경계·공통 topic/frame·GT-first 운영 순서는 [control_pipeline.md](control_pipeline.md)를, controller가 이 결과를 추종하는 방식은 [trajectory_follower.md](trajectory_follower.md)를, 실제 run 비교는 [trajectory_validator.md](trajectory_validator.md)를 따른다.

우선순위는 다음과 같다.

1. **competition route의 Lanelet 목록 CSV에서 해당 Lanelet 중심선을 이어 기준 trajectory를 만든다.**
2. 전체 코스 CSV면 전체 코스, 일부 Lanelet만 든 CSV면 그 순서의 부분 코스를 만든다.
3. RViz 클릭점 기반 Bézier 생성은 이번 구축에서 만들지 않는다. 정적 장애물 회피가 실제 scope가 됐을 때 별도 설계한다.

이 노드는 map/TF를 만들지 않는다. 기존 `maptf_morai.launch`, sensing, Lanelet map visualizer가 제공하는 `map` frame에서만 궤적과 marker를 publish한다.

## 2. 현재 확인한 competition route 자산

현재 워크스페이스에는 다음 CSV가 있다.

```text
kcity_map/competition_route/v44/drive_course_v44_lanelets.csv
```

확인된 특성:

- 60줄 중 header를 제외한 59행: `preferred_course` 42행, `candidate_corridor` 17행
- `preferred_course`에는 코스 종료부가 200889로 돌아오기 위한 Lanelet `200889`의 재등장이 포함됨. 이 행은 다음 lap 자동 시작을 뜻하지 않음
- 각 행에 `order`, `lanelet_id`, `role`, `section`, `successor_lanelet_id`, `speed_limit_kph`, `length_m`, `centerline_xy`가 있음
- `centerline_xy`는 `x:y:z;x:y:z;...` 형태의 이미 해석된 중심선이다.
- `drive_course_v44_summary.json`은 외부의 `lanelet2_map_v44.osm` 경로를 가리키지만, 이 워크스페이스에는 해당 OSM 파일이 없다.

따라서 v44 기본 코스의 1차 generator는 이 CSV의 `centerline_xy`를 직접 사용한다. 없는 OSM을 찾아 재투영하거나 MGeo 좌표계로 다시 변환하지 않는다.

## 3. 입력 모드

| Mode | 주 용도 | 우선순위 | 입력 |
|---|---|---:|---|
| `competition_catalog` | 전체 공식 코스 | 1 | `drive_course_v44_lanelets.csv` 같은 full catalog |
| `lanelet_selection` | 코스 일부/튜닝 구간 | 1 | 순서가 있는 Lanelet selection CSV + catalog |

### 3.1 Full catalog 입력

full catalog와 부분 selection 모두 `role=preferred_course`만 사용한다. `candidate_corridor`는 전체 코스 기준 궤적에 섞지 않고, selection CSV에 적혀도 1차 구현에서는 load 실패로 처리한다.

`order`가 기준 순서다. 동일 Lanelet ID가 두 번 있어도 제거하지 않는다. v44의 200889 재등장처럼 **한 번의 코스 완주를 위한** 의도된 재등장을 보존해야 하기 때문이다.

### 3.2 부분 코스 selection CSV

사람이 만들 CSV의 최소 형식은 아래와 같이 정한다.

```csv
order,lanelet_id
1,200311
2,200968
3,201095
```

속도까지 지정하는 권장 확장 형식:

```csv
order,lanelet_id,reverse,segment_name,target_speed_kph,note
1,200311,false,roundabout_entry,,튜닝 시작
2,200968,false,roundabout_entry,25,
3,201095,false,roundabout_entry,,
```

규칙:

- `order`, `lanelet_id`는 필수다. 파일 행 순서가 아니라 `order` 오름차순이 실제 연결 순서다.
- `lanelet_id`는 catalog의 `lanelet_id` 또는 `source_lanelet_id`와 정확히 일치해야 한다.
- `reverse` 기본값은 `false`다. generator가 "가까운 endpoint"라는 이유로 조용히 방향을 뒤집지 않는다.
- `target_speed_kph`가 비어 있으면 catalog의 `speed_limit_kph`를 **상한**으로 전달하고, 요청값은 profile YAML의 `constant_kph` 또는 `default_cruise_kph`를 쓴다.
- `target_speed_kph`는 해당 Lanelet에서 운용하고 싶은 **요청 목표속도**다. 이것은 규정 제한을 무시하는 override가 아니다. 최종 point target은 언제나 catalog의 `speed_limit_kph` 이하로 제한된다.
- 없는 Lanelet, 중복 `order`, 숫자가 아닌 ID, 연결 gap은 경고가 아니라 load 실패로 처리한다. 잘못된 trajectory를 controller에 보내는 것보다 수정할 위치를 즉시 아는 편이 낫다.

### 3.3 catalog와 selection의 관계

```text
full catalog
  lanelet_id -> centerline_xy, speed_limit, section, source metadata
       ▲
       │ lookup
selection CSV
  ordered lanelet_id list
       │
       ▼
Trajectory points in map frame
```

즉 selection CSV는 중심선 좌표를 중복 보관하지 않는 **경로 의도 파일**이다. map/version, full catalog path, checksum은 launch/config에 명시해 어떤 기준 geometry로 만든 trajectory인지 남긴다.

## 4. Lanelet 중심선 생성 알고리즘

### 4.1 처리 순서

1. `route_id`, `map_version`, catalog path, selection path를 읽는다.
2. full catalog mode면 `preferred_course` 행을 `order` 기준으로 선택한다. selection mode면 selection의 Lanelet IDs를 같은 순서로 catalog에서 찾는다.
3. 선택된 각 행의 `centerline_xy`를 `(x, y, z)` 배열로 parse한다.
4. selection의 `reverse=true`만 해당 lanelet point 순서를 반전한다.
5. 인접 lanelet의 이전 끝점과 다음 시작점 gap을 계산한다.
6. `stitch_gap_m`보다 큰 gap은 실패로 보고 `order`, 이전/다음 lanelet ID, gap 거리, endpoint를 marker와 report에 남긴다. 임의 직선 연결은 하지 않는다.
7. 정확히 겹치는 경계 point는 하나로 합친 뒤, 전체 polyline을 일정 arc length 간격으로 resample한다.
8. tangent로 yaw, finite difference로 curvature, curvature와 wheelbase로 예상 조향각을 계산한다.
9. 각 resample point에 catalog의 규정 `speed_limit_kph`를 펼치고, 요청 목표속도·곡률·가감속/jerk 한계를 적용해 point별 speed profile을 만든다.
10. 최종 `longitudinal_velocity_mps`, `acceleration_mps2`, `time_from_start`를 모든 `TrajectoryPoint`에 채운다.
11. full `Trajectory`, `Path`, route lanelet ID 목록, marker, load report를 publish한다.

### 4.2 출발/종료 및 loop 정책

competition route의 기준은 **반복 loop가 아닌 한 번의 완주**다.

- ego는 첫 번째 200889의 가장 아래쪽 시작점까지 수동으로 이동한다.
- generator는 첫 번째 200889 전체, 중간 `preferred_course` 전체, 마지막 200889 전체를 하나의 finite trajectory로 만든다.
- final point를 통과하면 velocity profile은 0으로 수렴하고 trajectory는 종료한다. 처음으로 자동 되돌아가 다음 lap을 시작하지 않는다.
- route config는 첫 occurrence와 마지막 occurrence를 구분해 아래처럼 고정한다. 200889의 `length_m`은 현재 catalog에서 72.629 m다.

```yaml
route_id: competition_v44_preferred_once
loop: false
start:
  lanelet_id: 200889
  occurrence_order: 1
  s_m: 0.0                 # 가장 아래쪽 시작점
finish:
  lanelet_id: 200889
  occurrence_order: 42
  s_m: 72.629              # 마지막 200889의 끝
```

따라서 lanelet trimming은 이 기본 route에서 하지 않는다. 향후 부분코스 selection에서만 start/finish `s_m` trim을 옵션으로 둔다.

### 4.3 launch-time 검증 항목

- catalog/selection 파일 존재 및 `map_version` 일치
- 선택 Lanelet의 누락 여부
- `centerline_xy` parse 가능 여부와 point 개수
- lanelet 간 endpoint gap
- 결과 length, point count, min/max point spacing
- yaw discontinuity, curvature, 예상 steering angle
- `speed_limit_kph`와 요청 `target_speed_kph`가 차량/규정 한계와 모순되지 않는지
- 모든 trajectory point의 target velocity가 해당 point의 규정 제한 이하인지
- 감속 경계 전부터 final stop까지 속도·가속도·jerk 제약을 만족하는지
- finite route의 final stop point와 stop velocity profile

RViz에는 정상 route를 굵은 색 선으로, failure lanelet과 endpoint gap을 빨간 marker/text로 보인다. controller를 켜기 전 generator만 launch해 이 검증이 통과하는 것이 2단계 완료 기준이다.

### 4.4 output topic

| Topic | Type | 목적 |
|---|---|---|
| `/planning/trajectory/reference` | `autoware_planning_msgs/Trajectory` | controller 공식 입력 |
| `/planning/trajectory/reference_path` | `nav_msgs/Path` | RViz 및 단순 도구 호환 |
| `/planning/trajectory/reference_markers` | `visualization_msgs/MarkerArray` | Lanelet segment, 시작/종료, 오류, speed/section 표시 |
| `/planning/trajectory/route_lanelet_ids` | `std_msgs/Int64MultiArray` | 선택된 실제 Lanelet ID 순서 |
| `/planning/trajectory/generator_status` | diagnostics 또는 status msg | load 결과와 error reason |

모든 geometry output은 `header.frame_id: map`이다. publish 주기와 trajectory point 간격은 다른 개념이다. 전체 route는 latch 또는 저주기 publish할 수 있으나, controller는 동일 기준 trajectory snapshot을 받는다.

매 load 때 generator는 `artifacts/trajectory_generator/<route_id>/<profile_id>/profile_report.csv`를 남긴다. 최소 열은 `s_m,x_m,y_m,z_m,yaw_rad,lanelet_id,speed_limit_kph,requested_speed_kph,curvature,target_speed_mps,acceleration_mps2,time_from_start_sec`다. `trajectory_validator`는 실제 run 시작 시 이 결과를 run의 `reference.csv`로 snapshot 복사한다. 따라서 generator가 만든 geometry·규정·요청·최종 속도와 실제 주행 결과를 같은 run에서 비교할 수 있다.

## 5. 속도 profile — 규정 속도를 trajectory point에 넣는 방법

기준 trajectory는 **어디를 지나갈지(geometry)** 와 **각 지점을 얼마나 빠르게 지나갈지(speed profile)** 두 부분으로 구성된다. Lanelet 중심선이 같아도 speed profile이 다르면 제어기가 받는 목표속도/가속도가 달라진다.

`speed_limit_kph`는 Lanelet마다 붙은 **이산적인 규정 제한속도 상한**이다. 예를 들어 한 lanelet은 60 km/h, 다음 lanelet은 100 km/h일 수 있다. generator는 이 값을 단순 참고값으로 버리지 않는다. resample된 **모든 trajectory point**에 소속 Lanelet의 규정 상한을 붙이고, 이를 넘지 않는 `longitudinal_velocity_mps`를 만들어 `TrajectoryPoint`에 넣는다.

`Trajectory` message에는 point별 규정 제한속도 metadata field가 없으므로, controller 공식 입력에는 최종 목표속도 `longitudinal_velocity_mps`와 `acceleration_mps2`를 넣는다. point별 raw `speed_limit_kph`, 요청값, 최종값은 generator의 `profile_report.csv`와 RViz speed marker에 함께 저장하고, validator가 이를 run의 `reference.csv`로 복사한다. 따라서 나중에 "왜 이 지점에서 30 km/h인가"를 원본 규정값까지 추적할 수 있다.

Lanelet limit을 point target에 그대로 복사하면 lanelet 경계에서 속도가 계단처럼 바뀌어 PI가 불필요하게 큰 accel/brake를 낸다. generator의 profile 단계는 이산 규정값을 입력으로 받아, **감속은 제한속도 하강 경계 전에 시작하고 가속은 상승 경계 뒤부터 시작하도록** 연속적인 목표속도로 바꾼다. 즉 "Lanelet 속도 정보라서 이산적"인 것이 맞고, profile 단계가 그 계단을 제어 가능한 속도 변화로 바꾸는 역할을 한다.

### 5.1 mode별 속도 입력

각 point `i`에서 kph를 m/s로 변환한 뒤 아래 상한을 만든다. `v_request`의 출처는 mode마다 다르다.

```text
v_legal[i] = catalog speed_limit_kph of point i
v_curve[i] = sqrt(max_lateral_accel_mps2 / max(abs(curvature[i]), epsilon))

mode=constant:
  v_request[i] = constant_kph

mode=regulation_curvature:
  v_request[i] = selection CSV target_speed_kph, or default_cruise_kph

both modes:
  v_cap[i] = min(v_legal[i], v_request[i], v_curve[i], vehicle_max_speed)
```

- `v_legal`은 catalog의 규정값이며, selection CSV가 이를 올릴 수 없다.
- `v_request`는 사용자가 넣는 운용 속도다. `regulation_curvature`에서 일반도로 40, 고속주회로 80을 넣어도 Lanelet limit이 60이면 최종 상한은 60이다.
- `v_curve`는 곡선에서 횡가속 한계를 넘지 않게 하는 상한이다. 직선에서는 매우 큰 값으로 취급해 규정/요청 속도가 선택된다.
- `constant` mode에는 selection CSV의 `target_speed_kph`를 섞지 않는다. 값이 있으면 load 실패로 알려준다. 그렇지 않으면 "고정 속도 PP tuning"과 "구간별 speed tuning"이 한 run에 섞여 비교가 무의미해진다.
- `constant_kph`가 일부 Lanelet limit보다 높으면 generator는 load 실패로 알려주거나, 명시한 `clamp_to_legal: true` 정책에서만 해당 구간을 낮춘다. 기본 정책은 tuning 실수를 빨리 찾을 수 있도록 **load 실패**다.

### 5.2 smoothing 알고리즘 — 기본으로 적용

generator는 아래 순서로 profile을 생성한다. 이 규칙은 사용자가 추가로 계산하지 않아도 기본 적용한다.

1. 중심선을 일정 `ds`로 resample하고, point마다 `v_cap`을 만든다.
2. 마지막 point는 `v=0`으로 고정한다.
3. **backward deceleration pass**: 끝에서 시작점 방향으로 훑으며 `v[i] <= sqrt(v[i+1]^2 + 2 * max_decel * ds)`를 적용한다. 제한속도 하강·급곡선·종료점 전에 충분히 감속한다.
4. **forward acceleration pass**: 시작점에서 끝 방향으로 훑으며 `v[i+1] <= sqrt(v[i]^2 + 2 * max_accel * ds)`를 적용한다. 제한속도 상승 뒤에도 즉시 jump하지 않고 가능한 가속도로 증가한다.
5. 실제 point timestamp로 가속도와 jerk를 계산하고, jerk 한계를 넘으면 주변 speed를 더 낮추는 forward/backward pass를 반복한다. 이 과정은 `v_cap`을 올리지 않고 낮추기만 하므로 규정 제한을 넘지 않는다.
6. 최종 `v[i]`에서 `acceleration_mps2`와 `time_from_start`를 다시 계산해 `TrajectoryPoint`에 기록한다.

이 방식으로 60→30 km/h 경계라면 60 구간의 마지막 직선부터 target이 `60 → 50 → 40 → 30`처럼 내려가며, 30→60 km/h 경계라면 경계를 지난 후 `30 → 40 → 50 → 60`으로 올라간다. 어느 point도 자기 위치의 규정 `v_legal`을 넘지 않는다.

### 5.3 profile 설정 파일

route마다 속도 의도를 CSV에 쓰고, smoothing 제약은 YAML로 둔다. 예시는 아래와 같다.

```yaml
speed_profile:
  mode: regulation_curvature       # constant | regulation_curvature
  default_cruise_kph: 40.0         # selection CSV가 비어 있을 때의 요청값
  constant_kph: 20.0               # mode=constant일 때만 사용
  clamp_to_legal: false            # false면 규정 초과 constant 요청은 load 실패
  vehicle_max_speed_kph: 80.0
  max_lateral_accel_mps2: 1.5
  max_accel_mps2: 1.0
  max_decel_mps2: 1.5
  max_jerk_mps3: 0.8
  final_stop_decel_mps2: 1.5
```

위 가감속·jerk 값은 첫 PP pipeline을 위한 보수적 시작값이다. validator의 `long_acc`, jerk, speed error, accel/brake saturation을 보며 run별로 하나씩 조정한다. `vehicle_max_speed_kph: 80`은 이번 운용 목표의 상한이며 Lanelet 규정값보다 우선하지 않는다.

### 5.4 `constant` profile

모든 point에 같은 **요청** 목표속도를 넣는다. 예를 들어 20 km/h로 설정하면 직선, 곡선 모두 controller는 우선 20 km/h를 유지하려 한다. 단, 규정 상한·final stop·선택한 curvature 제한은 여전히 적용된다.

```text
reference geometry:  [같은 Lanelet 중심선]
target velocity:     20, 20, 20, 20, 20 km/h
```

장점은 PP lookahead나 PI gain을 바꿨을 때 "속도 목표가 바뀌어서"가 아니라 제어 파라미터 효과만 비교하기 쉽다는 점이다. 따라서 **파이프라인/기본 PP+PI의 첫 튜닝은 `constant`를 권장**한다. 단, 지정 속도는 선택 lanelet의 `speed_limit_kph`보다 높게 설정하지 않는다.

### 5.5 `regulation_curvature` profile

각 Lanelet의 `speed_limit_kph`와 곡선의 curvature를 사용해 point마다 다른 목표속도를 만든다.

```text
v_cap(s) = min(user_cruise_speed, lanelet_speed_limit(s), curvature_speed_limit(s))
```

예를 들어 user cruise가 40 km/h일 때, 60 km/h 제한의 직선에서는 40 km/h를 유지하지만 급한 곡선에서는 횡가속 제한 때문에 20 km/h 이하로 내려간다. 이후 acceleration/deceleration/jerk 한계를 적용해 속도가 갑자기 튀지 않게 한다.

```text
straight:  40, 40, 40 km/h
curve:     40 → 30 → 20 → 30 → 40 km/h
```

이 방식은 실제 주행에 더 가깝지만, 종 제어 tuning과 횡 제어 tuning의 변수가 동시에 늘어난다. 따라서 기본 pipeline 검증이 끝난 뒤 적용한다. 신호등 정지와 정지선은 현재 Lanelet 속도/곡률만으로 자동 생성할 수 없으며, 향후 planner 또는 stop-zone 입력이 필요하다.

초기 profile mode 후보:

| Mode | 동작 | 용도 |
|---|---|---|
| `constant` | `constant_kph` + 규정/종료/곡률 cap | PP 기본 튜닝 |
| `regulation_curvature` | selection의 `target_speed_kph` 또는 default cruise, 규정, 곡률 상한의 min + smoothing | 실제 코스/구간별 속도 주행 |

전/후방 acceleration, deceleration, jerk 한계는 profile 생성에 적용한다. 전체 preferred route는 마지막 200889 끝에서 정지하고 loop하지 않는다.

## 6. 이번 구축에서 제외: RViz 클릭 Bézier mode

RViz 클릭 기반 Bézier 생성은 정적 장애물 회피용으로 유용하지만, 지금은 구현하지 않는다. 현재 목표는 Lanelet selection CSV만으로 전체/부분 기준 궤적을 쉽고 정확하게 만들고, 기본제어·validator 반복 튜닝을 먼저 완성하는 것이다.

향후 이 기능을 추가할 때에도 map frame의 클릭점, anchor/curve preview, Lanelet 복귀점, 경계/충돌 검증을 별도 설계한다. 이번 패키지 skeleton과 launch에는 `/clicked_point` subscriber를 넣지 않는다.

## 7. 제안 패키지 구성

```text
trajectory_generator/
  scripts/
    trajectory_generator_node.py
    competition_route_loader.py
    centerline_stitcher.py
    trajectory_profile.py
  config/
    routes/
      competition_route_catalogs.yaml
      selections/
    vehicle_limits.yaml
    generator_defaults.yaml
  launch/
    trajectory_generator.launch
  docs/
```

실행마다 바뀌는 route selection, speed mode, smoothing 제한은 이 package에 별도 run YAML을 만들지 않고 `purepursuit_pi/config/pp_pi.yaml`의 `generator` section에서 읽는다. 위 `generator_defaults.yaml`은 알고리즘 기본값만 담으며, PP/PI gain tuning 때 사용자가 반복 수정하지 않는다.

이 구조는 기존 `ajou_control/mgeo_trajectory_generator.py`의 path/marker 사용 경험은 참고하되, MGeo `link_set.json`과 coordinate offset을 공식 기준으로 승계하지 않는다. 공식 기준 geometry는 competition-route catalog의 `map` 좌표다.

## 8. 구현 acceptance checklist

### 8.1 반드시 통과할 규칙

1. selection CSV는 `preferred_course` Lanelet만 허용한다. `candidate_corridor`는 1차 scope에서 제외한다.
2. custom CSV의 Lanelet ID가 지정 catalog에 없으면 **실패**한다. 다른 map version을 자동 추정하거나 geometry를 만들어 내지 않는다.
3. lanelet 간 gap/방향 불일치도 **실패**한다. graph route를 자동 보완하거나 임의 직선으로 잇지 않는다.
4. 전체 코스는 **loop하지 않는다**. order 1의 200889 시작점에서 출발해 order 42의 200889 끝에서 정지한다.
5. 첫 PP tuning route/속도/spacing은 pipeline이 동작함을 보이는 예시로 선택한다. 이 선택은 구현 시 현재 map과 MORAI start pose를 보고 정한다.

### 8.2 PP 속도 운용 순서

1. 첫 PP 직선/완만한 곡선 pipeline run은 `constant_kph: 20.0`으로 한다. 이 값도 point별 `longitudinal_velocity_mps`로 채워지고, 규정·곡률·종료 제약을 통과해야 한다.
2. 같은 route에서 PP가 정상 start/finish를 내고, validator에 control deadline miss·trajectory stale·규정 초과 target이 없는 run을 최소 3회 남기면 `regulation_curvature`로 전환한다.
3. 전환 뒤 기본 요청속도는 `default_cruise_kph: 40.0`이다. selection CSV의 `target_speed_kph`로 구간별 25/40/80 등 운용 의도를 넣되, 어떤 경우에도 catalog `speed_limit_kph`와 곡률/가감속/jerk 상한을 넘지 않는다.
4. 고속주회로 80 km/h 요청은 PP의 40 km/h profile 검증과 해당 구간의 steering/longitudinal validator 결과를 확인한 뒤에만 추가한다.
