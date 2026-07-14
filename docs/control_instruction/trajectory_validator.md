# `trajectory_validator` 설계 지시서

## 1. 역할

`trajectory_validator`는 제어기 앞에서 trajectory를 차단하는 gate가 아니다. **기준 궤적과 실제 주행 궤적의 차이를 직관적으로 보고, 반복 튜닝을 빠르게 하는 평가 패키지**다.

종/횡 제어가 다르므로 결과도 종·횡으로 분리해 기록·시각화한다. 한 run의 원자료와 파라미터를 보존하고, 이전 run을 선택해 겹쳐 보이게 하는 것이 필수 기능이다.

이 문서는 평가와 artifact의 기준이다. 공통 topic/frame 및 GT-first 운영은 [control_pipeline.md](control_pipeline.md), PP 제어 파라미터의 뜻은 [trajectory_follower.md](trajectory_follower.md), reference 생성과 profile report는 [trajectory_generator.md](trajectory_generator.md)를 따른다.

## 2. input/output

| 방향 | Topic/파일 | Type | 용도 |
|---|---|---|---|
| 입력 | `/planning/trajectory/reference` | `autoware_planning_msgs/Trajectory` | target pose/speed/accel와 route identity |
| 입력 | `/planning/trajectory/generator_status` | latched `std_msgs/String` | generator가 실제 생성한 route/profile report metadata |
| 입력 | `/control/purepursuit_pi/ego_state` | `nav_msgs/Odometry` | follower가 정규화한 실제 map pose, body velocity, timestamp |
| 입력 | `/ctrl_cmd` | `morai_msgs/CtrlCmd` | MORAI command steering/longitudinal input |
| 입력 | `/control/purepursuit_pi/run_config` | latched `std_msgs/String` | 재시작된 PP controller가 실제로 읽은 config/run label/checksum snapshot |
| 출력 | `/validation/lateral/*` | `std_msgs/Float32` | 횡 시계열 |
| 출력 | `/validation/longitudinal/*` | `std_msgs/Float32` | 종 시계열 |
| 출력 | `/validation/metrics/*` | `std_msgs/Float32` | 누적/최종 지표 |
| 출력 | `/validation/markers` | `visualization_msgs/MarkerArray` | RViz target/current/history overlay |
| 출력 | run artifact directory | CSV/YAML | 오프라인 비교·재현 |

평가 노드는 controller와 독립된 observer다. evaluator가 내려가도 controller를 멈추게 하지 않으며, controller의 safety stop 판단도 대신하지 않는다.

### 현재 PP validation source — follower normalized ego state

validator는 MORAI/localization toggle을 자체 YAML에 복제하지 않는다. **현재 PP pipeline은 follower의 `ego_state.source: morai_gt`**로 `/Ego_topic`과 `map -> base_link` TF를 해석하고, 그 결과를 `/control/purepursuit_pi/ego_state`로 publish한다.

- validator는 위 Odometry의 `map` pose, orientation, body-frame long/lat velocity와 timestamp만 사용한다.
- 첫 PP run의 target-vs-driven path, lat/heading/speed error, ADE/FDE는 follower가 정규화한 MORAI GT 기준이다.
- localization 전환 시에도 validator YAML을 바꾸지 않는다. follower의 source만 바꾸고 새 run snapshot의 `ego_state_mode`로 구분한다.

### 후속 localization source — 계약만 보존

`localization` mode는 follower가 **`/localization/odom` (`nav_msgs/Odometry`)**과 같은 timestamp의 `map -> base_link` TF를 검증한 뒤 normalized ego topic으로 내보내는 후속 mode다. validator는 raw FAST-LIO frame을 직접 해석하지 않는다. source와 차량 상태 계약은 controller run snapshot에 기록한다.

## 3. RViz 평가 화면

기존 Lanelet map, `maptf_morai`, sensing/vehicle RViz 위에 다음 display만 추가한다. 모든 marker는 `map` frame이다.

`trajectory_validator.rviz`는 시작부터 K-City route 범위가 보이도록 top-down overview를 사용한다. 이전 설정처럼 원점 주변의 기본 camera를 쓰지 않는다. `/map/lanelet_map` publisher가 이미 실행 중이면 map layer도 같은 화면에 표시하며, map publisher는 validator가 소유하지 않는다.

| layer | 색/형태 | 의미 |
|---|---|---|
| Lanelet map | 기존 display | 맵/차선 문맥 |
| target trajectory | 청록 또는 파랑 굵은 선 | 이번 run의 reference trajectory |
| current driven path | 노랑 굵은 선 | 현재 controller가 실제로 지나간 궤적 |
| selected history runs | run별 서로 다른 반투명 색 | 비교 대상 과거 주행 궤적 |
| nearest projection / error vector | 흰 점 + 빨강 선 | 순간 lateral error가 어디서 생겼는지 |
| PP lookahead / local reference | controller debug 색 | 횡제어가 실제로 본 target |
| start / finish | 초록 / 빨강 | run 범위와 FDE 기준 |

### 3.1 history를 클릭해서 켜고 끄는 방식

history는 자동으로 모든 과거 run을 불러오지 않으며, launch argument를 매번 고쳐서 선택하지도 않는다. RViz 기본 UI에는 artifact index의 run 목록을 checkbox로 표시할 기능이 없으므로, `trajectory_validator`가 제공하는 **rqt history panel**에서 선택한다.

```text
rqt trajectory-validator history panel

Route: competition_v44_preferred_once     [Reload]
Filter: [complete only] [same route/map/frame/source/reference only]

[ ] pp_ld50_kp100_ki30   20260714_143000   lat_rms=0.21 m
[x] pp_ld60_kp100_ki30   20260714_150200   lat_rms=0.18 m
[x] pp_ld70_kp100_ki30   20260714_153500   lat_rms=0.25 m
```

- 기본 선택은 **없음**이다. 현재 주행 path와 target trajectory만 RViz에 보인다.
- 사용자가 row의 checkbox를 누르면 panel이 `/trajectory_validator/history/set_visible`에 `run_id, visible`을 보낸다. viewer node가 해당 run의 `samples.csv`를 읽어 `history/<run_id>` marker namespace로 publish한다.
- checkbox를 해제하면 그 run namespace의 marker만 `DELETE`한다. `DELETEALL`은 target/current marker까지 지울 수 있으므로 사용하지 않는다.
- index는 `index.csv`에서 읽고, route/map/frame/control-rate/ego-state mode/reference checksum이 다른 run은 기본 filter에서 숨긴다. 필요하면 filter를 풀어 경고 색으로 표시할 수 있다.
- 선택한 history run은 메모리에 cache해 다시 체크할 때 파일을 다시 읽지 않는다. panel을 닫거나 validator를 재시작하면 기본적으로 모두 해제한다.

service 계약은 아래처럼 작게 둔다.

```text
# SetHistoryVisibility.srv
string run_id
bool visible
---
bool success
string message
```

따라서 사용자는 rqt에서 원하는 과거 run만 클릭해 RViz에 띄우고, 같은 checkbox를 해제해 즉시 없앨 수 있다. history panel의 위치는 rqt perspective에 포함하고, plot topic 구성은 별도 `rqt_multiplot` XML에 둔다.

## 4. rqt_multiplot 필수 항목

### 4.1 횡 검증

| Topic | 단위 | 정의 |
|---|---:|---|
| `/validation/lateral/lat_err` | m | reference polyline nearest **projection** 기준 signed lateral error |
| `/validation/lateral/heading_err` | rad | projection 지점 tangent yaw - ego yaw의 wrap된 차이 |
| `/validation/lateral/steering_angle_cmd` | rad | `/ctrl_cmd.front_steer` command |
| `/validation/lateral/lat_vel` | m/s | ego body frame lateral velocity |
| `/validation/lateral/lat_acc` | m/s² | lateral velocity의 timestamp 기반 미분 |

추가 권장: `steering_rate_cmd`, `curvature_ref`, `lookahead_distance`, `lat_err_abs`, `lat_err_rms`.

### 4.2 종 검증

| Topic | 단위 | 정의 |
|---|---:|---|
| `/validation/longitudinal/long_vel` | m/s | ego body frame longitudinal velocity |
| `/validation/longitudinal/target_vel` | m/s | trajectory projection 시점의 target velocity |
| `/validation/longitudinal/long_vel_err` | m/s | target - actual |
| `/validation/longitudinal/long_acc` | m/s² | longitudinal velocity의 timestamp 기반 미분 |
| `/validation/longitudinal/target_acc` | m/s² | trajectory acceleration |
| `/validation/longitudinal/accel_cmd` | 0..1 | 규정 type 1의 `/ctrl_cmd.accel` pedal command |
| `/validation/longitudinal/brake_cmd` | 0..1 | 규정 type 1의 `/ctrl_cmd.brake` pedal command |

추가 권장: `jerk`, `progress_s_m`, `remaining_distance_m`, brake/acceleration saturation, stop-state.

### 4.3 run 지표

| Topic | 정의 |
|---|---|
| `/validation/metrics/mean_ade` | 동기화된 각 ego sample에서 reference polyline까지의 거리 평균 |
| `/validation/metrics/fde` | run 종료 시 actual final pose와 reference final pose 사이 거리 |
| `/validation/metrics/lateral_rms` | lateral error RMS |
| `/validation/metrics/heading_rms` | heading error RMS |
| `/validation/metrics/speed_rms` | longitudinal velocity error RMS |
| `/validation/metrics/max_lat_err` | 최대 절대 lateral error |
| `/validation/metrics/p95_lat_err` | 95 percentile absolute lateral error |
| `/validation/metrics/steer_saturation_rate` | steering command가 한계 근처인 sample 비율 |
| `/validation/metrics/control_deadline_miss_count` | controller timer deadline miss 누적 |

`FDE`는 매 sample의 "경로 끝까지 남은 거리"가 아니다. 후자는 `/validation/longitudinal/remaining_distance_m`로 분리한다. FDE는 finite trajectory의 configured finish 조건이 만족된 마지막 actual pose와 reference 종료 pose의 endpoint error로 계산한다.

## 5. sample 계산 원칙

1. ego timestamp 기준으로 reference polyline에 projection한다. 단순 nearest vertex가 아니라 line segment projection을 사용한다.
2. projection arc length `s`에서 reference position, tangent yaw, velocity, acceleration을 interpolate한다.
3. ego pose를 body frame 기준 속도로 분해해 long/lat velocity를 구한다.
4. acceleration은 실제 message timestamp 차이로 계산한다. `dt`가 비정상 또는 stale이면 sample을 flag하고 미분값은 신뢰하지 않는다.
5. `/ctrl_cmd`의 `longlCmdType`, `front_steer`, `accel`, `brake`, `velocity`, `acceleration`을 raw sample에 함께 남긴다. 대회용 run은 `longlCmdType: 1`만 정상으로 보고, 현재 MORAI 단계에서는 command value를 평가하며 feedback topic을 가정하지 않는다.
6. progress가 역행하거나 큰 jump가 나면 이를 별도 event로 기록한다. 이번 competition route는 loop하지 않으므로 lap index를 만들지 않는다.

## 6. 자동 start/finish — 누르는 절차 없음

validator는 수동 start/stop command나 service를 요구하지 않는다.

```text
ARMED
  └─ ego가 첫 200889의 start point 3 m 이내, heading tolerance 이내에 들어오고
     speed가 start_speed_mps 이상이면 자동 START

RECORDING
  └─ final progress 3 m 이내이고, 마지막 200889의 끝 3 m 이내,
     heading tolerance 이내에 도달하고
     speed가 finish_speed_mps 이하이면 자동 FINISH
```

- MORAI에서 사람이 차량을 start gate 부근까지 수동으로 이동한 뒤 출발하면 된다. gate에 들어가기 전 수동 reposition 구간은 기록하지 않는다.
- finish는 order 42 마지막 200889의 끝에서만 판정한다. loop/lap 재시작은 없다.
- 첫 PP pipeline의 gate 초기값은 아래와 같다. rear axle 기준 reference point와 수동 reposition 오차를 고려해 3 m를 쓴다. 1 m는 TF/수동 배치 오차에 너무 민감하고, 5 m는 finish 위치/FDE가 너무 느슨해져서 쓰지 않는다.

```yaml
validation_gate:
  start_distance_m: 3.0
  finish_distance_m: 3.0
  start_heading_tolerance_deg: 45.0
  finish_heading_tolerance_deg: 45.0
  start_speed_mps: 0.5
  finish_speed_mps: 1.0
```

- finish는 거리만으로 판정하지 않고 final trajectory progress가 마지막 3 m 안에 있어야 한다. start/finish 거리, heading tolerance, speed threshold는 route config에 기록하고 run metadata에도 복사한다. 3회 정도의 실제 run에서 start 누락 또는 조기 finish가 반복될 때만 같은 route config에서 값 하나씩 조정한다.
- route 중간에서 멈추거나 controller가 safe stop이 되면 `aborted` event와 당시 지표를 저장한다. 정상 완주 run과 같은 성능 비교 대상으로 표시하지 않는다.

## 7. validator history와 run 비교 방식

```text
base_ws/src/control/trajectory_validator/
  history/
    pp_pi/
      index.csv
      <run_id>/                 # <YYYYmmdd_HHMMSS>__<run_label>
        metadata.yaml
        parameters.yaml         # 실행 시작 순간의 pp_pi.yaml 실제 적용값
        reference.csv           # generator profile report의 불변 snapshot
        samples.csv             # 실제 주행/추종/command 시계열 원자료
        metrics.yaml
        events.csv              # start/finish/abort/stale 등 이산 이벤트
```

현재 controller는 PP+PI 하나이므로 모든 run은 `history/pp_pi/` 아래에 저장한다. route ID는 폴더를 더 나누는 기준이 아니라 `metadata.yaml`과 `index.csv`의 filter field로 사용한다. 나중에 `mpc_pi`를 실제 구현할 때만 `history/mpc_pi/`를 새로 추가한다.

`trajectory_validator/history/.gitignore`는 `.gitignore`와 `.gitkeep`을 제외한 모든 runtime artifact를 ignore한다. 따라서 history의 CSV/YAML run 결과는 로컬 비교용으로만 보관하고 GitHub에는 올리지 않는다.

validator가 직접 수정하는 파일은 `trajectory_validator/config/trajectory_validator.yaml`이며 gate/history/timeout만 담는다. validator는 generator YAML과 PP YAML을 읽지 않는다. `pp_pi.launch`가 재시작될 때 latched `/control/purepursuit_pi/run_config`으로 받은 실제 controller config snapshot을 `parameters.yaml`으로 복사하고, generator status에서 route/profile identity를 받는다. 따라서 generator/validator를 유지한 채 controller만 재시작해도 history에는 실제 적용된 값이 남는다. `RECORDING` 중 config snapshot이 바뀌면 validator는 해당 run을 `aborted`로 확정하고 새 config를 다음 `ARMED` run부터만 받는다.

### 7.1 CSV 기록 원칙 — 확정

수치 시계열의 canonical artifact는 **CSV**로 한다. JSON은 이 pipeline의 run 원자료로 만들지 않는다. CSV는 pandas/Excel/gnuplot과 rqt 결과 대조에 바로 쓸 수 있고, 두 run을 `stamp_sec` 또는 `ref_s_m`로 join하기 쉽다. `metadata.yaml`, `parameters.yaml`, `metrics.yaml`은 계층형 설정·요약만 담는다.

- 모든 CSV는 UTF-8, comma delimiter, 첫 행 header, SI 단위, 소수점 `.`을 사용한다. 결측 numeric 값은 `NaN`이 아니라 빈 칸으로 기록하고, 원인은 flag column 또는 `events.csv`에 남긴다.
- `samples.csv`는 `RECORDING` 상태에서 **ego pose message timestamp마다 한 행**을 쓴다. 첫 PP 단계에서는 nominal 20 Hz이며, validator timer가 임의로 보간해 행을 만들지 않는다.
- 각 sample에는 그 ego timestamp보다 같거나 이전인 가장 최근 `/ctrl_cmd`를 붙인다. command가 없거나 오래되면 값은 비우고 `ctrl_valid=0`, 원인은 event로 남긴다.
- `reference.csv`는 run 시작 때 사용한 generator `profile_report.csv`를 그대로 복사한 snapshot이다. run 중 generator 설정/파일이 바뀌어도 이미 시작한 run의 reference는 바꾸지 않는다.
- CSV 기록은 `history/pp_pi/.in_progress_<run_id>/`에 누적한 뒤 finish 또는 abort 시 `history/pp_pi/<run_id>/`로 확정한다. raw `samples.csv`를 요약값으로 덮어쓰지 않으며, `metrics.yaml`은 이 raw data에서 다시 계산 가능해야 한다.

### 7.2 `samples.csv` — 실제 주행·추종 시계열

한 행은 normalized ego Odometry timestamp의 실제 차량 상태와 그 시점에 reference에 projection한 결과다. `ego_*`는 follower가 publish한 `map -> base_link` 기준 값이며, 초기 PP run에서는 MORAI GT `/Ego_topic`을 follower가 정규화한 결과다. `ref_*`는 reference polyline의 segment projection 및 interpolation 결과다.

```csv
sample_index,stamp_sec,dt_sec,ego_x_m,ego_y_m,ego_z_m,ego_yaw_rad,ref_s_m,ref_x_m,ref_y_m,ref_z_m,ref_yaw_rad,ref_lanelet_id,ref_curvature_1pm,ref_speed_mps,ref_acceleration_mps2,remaining_distance_m,lat_err_m,heading_err_rad,ref_distance_m,long_vel_mps,lat_vel_mps,long_acc_mps2,lat_acc_mps2,ctrl_stamp_sec,ctrl_age_sec,longl_cmd_type,front_steer_rad,steering_rate_cmd_radps,accel_cmd,brake_cmd,velocity_cmd_mps,acceleration_cmd_mps2,lookahead_distance_m,control_tick_elapsed_ms,control_deadline_miss,ego_valid,reference_valid,ctrl_valid,projection_valid
```

| 열 | 단위 | 확정 정의 |
|---|---:|---|
| `sample_index` | - | run 안에서 0부터 증가하는 기록 순번 |
| `stamp_sec`, `dt_sec` | s | ego ROS timestamp와 직전 sample과의 실제 timestamp 차이 |
| `ego_x_m`, `ego_y_m`, `ego_z_m`, `ego_yaw_rad` | m, rad | `map -> base_link`의 실제 ego pose. yaw는 map x축 기준 반시계방향 |
| `ref_s_m` | m | 시작점부터 projection 지점까지의 reference arc length |
| `ref_x_m`, `ref_y_m`, `ref_z_m`, `ref_yaw_rad` | m, rad | `ref_s_m`에서 interpolate한 target pose |
| `ref_lanelet_id` | - | projection 지점이 속한 generator Lanelet ID |
| `ref_curvature_1pm` | 1/m | target trajectory의 signed curvature |
| `ref_speed_mps`, `ref_acceleration_mps2` | m/s, m/s² | 해당 target point의 최종 속도·가속도 profile 값 |
| `remaining_distance_m` | m | `reference_total_length - ref_s_m`; FDE가 아님 |
| `lat_err_m` | m | reference tangent의 left side를 양수로 한 signed 횡오차 |
| `heading_err_rad` | rad | `wrap(ego_yaw_rad - ref_yaw_rad)` |
| `ref_distance_m` | m | ego point에서 reference polyline까지의 2D 최단거리. `mean_ade` 재계산의 원자료 |
| `long_vel_mps`, `lat_vel_mps` | m/s | ego body frame의 전방·좌측 속도 |
| `long_acc_mps2`, `lat_acc_mps2` | m/s² | 실제 timestamp 차분으로 계산한 전방·좌측 가속도 |
| `ctrl_stamp_sec`, `ctrl_age_sec` | s | 붙인 최신 `/ctrl_cmd`의 timestamp와 `stamp_sec - ctrl_stamp_sec` |
| `longl_cmd_type` | - | 기록된 MORAI `CtrlCmd.longlCmdType`; 정상 PP run은 `1` |
| `front_steer_rad` | rad | 기록된 MORAI front steering command |
| `steering_rate_cmd_radps` | rad/s | 연속 command 두 개의 `front_steer_rad` 차분 / command timestamp 차이 |
| `accel_cmd`, `brake_cmd` | 0..1 | MORAI type 1 pedal command |
| `velocity_cmd_mps`, `acceleration_cmd_mps2` | m/s, m/s² | `CtrlCmd`의 raw velocity/acceleration field. type 1에서 controller가 쓰지 않으면 빈 칸 |
| `lookahead_distance_m` | m | 해당 PP control tick의 실제 lookahead 거리 |
| `control_tick_elapsed_ms` | ms | controller가 publish한 해당 tick 계산시간 diagnostic |
| `control_deadline_miss` | 0/1 | 해당 control tick이 설정 주기 deadline을 넘었는지 |
| `ego_valid`, `reference_valid`, `ctrl_valid`, `projection_valid` | 0/1 | 해당 행의 source freshness·reference·command·projection 유효 여부 |

`dt_sec`가 비정상, ego/reference가 stale, projection이 jump한 행도 삭제하지 않는다. 유효 flag와 `events.csv`를 남겨 나중에 데이터 제외 기준을 재현한다. 단, `projection_valid=0`이면 reference 기반 numeric 열은 빈 칸으로 기록한다.

### 7.3 `reference.csv`와 `events.csv`

`reference.csv`의 열은 generator가 확정한 아래 순서 그대로다. 이 파일의 SHA-256을 `metadata.yaml.reference_checksum`에 저장한다.

```csv
s_m,x_m,y_m,z_m,yaw_rad,lanelet_id,speed_limit_kph,requested_speed_kph,curvature,target_speed_mps,acceleration_mps2,time_from_start_sec
```

`events.csv`는 시계열 sample과 분리한 이산 상태 변화 기록이다.

```csv
event_index,stamp_sec,event_type,severity,progress_s_m,ego_x_m,ego_y_m,detail
```

| 열 | 정의 |
|---|---|
| `event_index`, `stamp_sec` | run 안의 event 순번과 발생 ROS timestamp |
| `event_type` | `armed`, `start`, `finish`, `aborted`, `ego_stale`, `reference_stale`, `ctrl_stale`, `projection_jump`, `progress_regression`, `safe_stop`, `control_deadline_miss` 중 하나 |
| `severity` | `info`, `warning`, `error` 중 하나 |
| `progress_s_m`, `ego_x_m`, `ego_y_m` | 발생 당시의 reference progress와 ego map 위치. 알 수 없으면 빈 칸 |
| `detail` | route ID, stale age, abort reason 등 사람이 읽을 수 있는 짧은 보조 설명. CSV quoting 규칙을 따른다. |

`history/pp_pi/index.csv`는 history 목록용 요약이며 다음 고정 열만 쓴다. PP/MPC별 세부 parameter는 이 파일에 중복하지 않고 `run_label`과 `parameters.yaml`에서 확인한다.

```csv
run_id,run_label,route_id,controller,map_version,frame_id,control_rate_hz,ego_state_mode,reference_checksum,created_at,status,mean_ade_m,fde_m,lateral_rms_m,heading_rms_rad,speed_rms_mps,artifact_relpath
```

`metadata.yaml` 최소 항목:

```yaml
run_label: pp_ld50_kp100_ki30
run_id: 20260714_143000__pp_ld50_kp100_ki30
route_id: competition_v44_full
map_version: v44
frame_id: map
controller: purepursuit_pi
control_rate_hz: 20
ego_state_topic: /control/purepursuit_pi/ego_state
ego_state_mode: morai_gt
vehicle_model: 2023_Hyundai_ioniq5
wheelbase_m: 3.000
morai_longlCmdType: 1
generator_profile_checksum: <sha256>
reference_checksum: <sha256>
start_condition: gate_3m_heading45_speed0.5
```

파일/디렉터리 이름에는 controller 이름과 사용자가 가장 많이 바꾸는 주요 parameter를 넣는다. PP의 사용자 표시 이름은 **`pp_<parameter><value>_...`**로 고정한다.

```text
pp_ld50_kp100_ki30
│  │    │     └─ Ki = 0.30  (value × 100)
│  │    └─────── Kp = 1.00  (value × 100)
│  └──────────── fixed lookahead = 5.0 m (value × 10)
└─────────────── purepursuit_pi controller label
```

- PP 첫 tuning은 fixed lookahead mode를 쓰므로 `ld`는 실제 lookahead 거리다. velocity-scaled lookahead를 도입하면 모호한 `ld`를 쓰지 않고 `ldmin50_ldg10_ldmax160`처럼 min/gain/max를 모두 이름에 넣는다.
- `run_label`은 실제 parameter snapshot의 `run_name_keys`를 정해진 순서와 scale로 조합해 **자동 생성**한다. 사람이 임의로 이름을 고쳐 parameter와 label이 어긋나지 않게 한다. `run_id`는 timestamp를 앞에 붙여 같은 파라미터 재주행도 덮어쓰지 않게 하며, RViz/rqt 목록은 짧은 `run_label`을 먼저 보인다.
- `steer_cap`, accel/brake gain 등 나머지 값은 항상 `parameters.yaml`에 전체 보존한다. 해당 값을 바꾸는 tuning부터는 `run_name_keys`에 추가해 `pp_ld50_kp100_ki30_sc35`처럼 label에도 넣는다.

`index.csv`에는 위 고정 header 순서대로 run 식별값, 비교 filter 값, 핵심 지표와 `history/pp_pi/` 기준 artifact 상대 경로를 한 행으로 남긴다. controller별 주요 parameter는 `run_label`과 `parameters.yaml`에만 두어 CSV header가 PP/MPC에 따라 바뀌지 않게 한다. rqt history panel은 이 index에서 run을 골라 읽는다.

```text
roslaunch trajectory_validator trajectory_validator.launch
```

즉 사람이 긴 CSV 경로를 직접 찾지 않고, rqt history panel에서 controller/parameter가 드러나는 run label을 checkbox로 골라 RViz에서 비교한다. `samples.csv`는 raw time-series를 요약으로 덮어쓰지 않고, `metrics.yaml`은 raw data에서 재계산 가능한 최종 요약이다.

## 8. 공정한 tuning 시작 조건

초기 PP tuning은 실제 map 위에서 한다. competition route 일부를 "공터"로 평행이동하면 Lanelet map, target trajectory, MORAI GT TF의 관계가 끊어져 실제 파이프라인 검증이 아니다.

- generator에서 실제 course의 단순한 짧은 직선/완만한 곡선 Lanelet selection CSV를 만든다.
- MORAI에서 차량을 해당 start gate 주변으로 수동 이동한다.
- vehicle이 start gate를 통과하며 움직일 때 validator가 자동 기록을 시작한다.
- 별도의 공터 시험이 정말 필요하면 map route를 옮긴 결과를 competition route와 섞지 않고 `synthetic_*` route ID로 별도 관리한다. 이번 구축에서는 만들지 않는다.

## 9. 고정 rqt 탭 layout

`traj_val`의 RQT panel은 종방향·횡방향·통합 tab을 동시에 제공한다. 사용자는 PP/PI YAML에 mode를 쓰지 않고 필요한 tab만 클릭한다. tab 선택은 주행·run name·history 저장에 영향을 주지 않으며, 같은 실행 중에도 바꿀 수 있다.

| tab | rqt에 표시할 시계열 | tuning 때 주로 보는 parameter |
|---|---|---|
| `longitudinal` | `target_vel`, `long_vel`, `long_vel_err`, `long_acc`, `accel_cmd`, `brake_cmd` | PP lookahead, steering limit/rate |
| `lateral` | `lat_err`, `heading_err`, `steering_angle_cmd`, `steering_rate_cmd`, `lat_vel`, `lat_acc`, `lookahead_distance` | PI `Kp/Ki`, accel/brake gain |
| `coupled` | `lat_err`, `steering_angle_cmd`, `target_vel`, `long_vel`, `long_acc`, `accel_cmd`, `brake_cmd`의 요약 | 새 gain 변경 없음. 종/횡 tuning 통과값 |

세 tab 모두 history checkbox panel을 함께 쓴다. CSV에는 tab과 무관하게 확정한 전체 raw column을 기록한다. 화면을 줄이는 것은 원인 분리를 위한 것이지, 저장 데이터를 줄이는 것이 아니다. RQT plot x축은 ROS epoch timestamp를 자동 추종하므로 빈 `0~10 s` 화면을 만들지 않는다.

## 10. 구현 단계

1. `Trajectory`, ego pose, command를 받아 reference/current path와 `lat_err`, `heading_err`, `long_vel`을 publish한다.
2. RViz에 target/current/start/finish를 표시하고 run directory에 reference/samples를 저장한다.
3. rqt의 필수 횡·종 topic과 ADE/RMS/FDE를 추가한다.
4. rqt history checkbox panel, 선택 run overlay, parameter snapshot, metrics comparison을 추가한다.
5. PP controller timing, 규정 type 1 `accel`/`brake` command, jerk/saturation, route completion event를 추가한다.

기존 `ajou_control/scripts/tracking_validator.py`의 rqt topic 목록, CSV logging, history overlay는 재사용 후보지만, 최종 패키지는 위 artifact schema와 `Trajectory` 계약으로 다시 정리한다.

## 11. 확정 사항과 후속 확인

1. PP validator gate는 start/finish 모두 3 m, heading 45°, start 0.5 m/s, finish 1.0 m/s로 시작한다.
2. history는 `trajectory_validator/history/pp_pi/`에 저장하며, 기본 선택은 none이다. rqt history panel의 checkbox로 원하는 run만 RViz에 표시·제거한다.
3. PP 기본 label은 `pp_ld50_kp100_ki30` codec을 쓴다. 바꾼 파라미터는 label에 같은 규칙으로 추가하고, 전체 값은 항상 `parameters.yaml`에 남긴다.
4. 현재 PP pipeline은 `source: morai_gt`, 즉 `/Ego_topic`으로 주행한다. localization switch와 FAST-LIO TF 검증은 후속 단계다.
