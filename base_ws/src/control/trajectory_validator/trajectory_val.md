# Trajectory Validator 설계 요약

## 목적과 모드

`trajectory_validator`는 follower 추종 결과를 저장하고 이전 run 하나를 다시 publish한다.
trajectory와 제어 명령은 생성하거나 수정하지 않는다.

| 모드 | 진입점 | 역할 |
| --- | --- | --- |
| Live record | `trajectory_validator.launch` | 현재 주행을 CSV/JSON으로 저장 |
| History replay | `publish_history.py` | 지정한 run 하나를 30 Hz로 재생 |

## Live record

입력:

| Topic | Type |
| --- | --- |
| `/control/trajectory` | `autoware_planning_msgs/Trajectory` |
| `/localization/kinematic_state` | `nav_msgs/Odometry` |
| `/Ego_topic` | `morai_msgs/EgoVehicleStatus` (차량 고유 telemetry) |
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` |
| `/CollisionData` | `morai_msgs/CollisionData` (선택) |

첫 유효 trajectory 전체를 GT로 한 번 저장하고 Ego 상태를 기본 30 Hz로 sampling한다. 위치,
자세, 속도는 `map -> base_link` 공통 kinematic state에서 읽고 가속도·휠각 등 MORAI 전용 값만
`/Ego_topic`에서 읽는다. trajectory와 state frame이 다르면 sample을 유효 처리하지 않는다.
`sample_valid`는 모든 입력이 fresh하고 같은 MORAI packet에서 파생된 Ego/state stamp 차이가
`sync_tolerance_s` 이내일 때만 true다. 정적 기준 trajectory의 stamp는 센서 동기 조건으로
사용하지 않는다. 통계는 valid sample만 사용한다.

collision, goal 도달 또는 ROS shutdown으로 run을 종료한다. 종료 시 CSV를 닫은 뒤 JSON을
`status=complete`로 원자적으로 갱신한다.
MORAI의 충돌 없음 placeholder(`ObjectStatus.type=-1`)는 collision으로 계산하지 않는다.

## 저장 구조

```text
history/<traj_sample>/<MMDD_HHMMSS_run_params>.csv
history/<traj_sample>/<MMDD_HHMMSS_run_params>.json
```

- `traj_sample`: 입력 trajectory CSV의 확장자를 제외한 basename
- 동일한 run ID가 있으면 `_01`, `_02` 순으로 새 이름 사용
- 이름은 `[A-Za-z0-9_-]`만 허용

CSV row type:

- `GT_POINT`: 전체 기준 trajectory의 point
- `SAMPLE`: Ego 상태, CtrlCmd, matched GT, 오차, collision/goal/valid 상태

JSON에는 frame, 입력 trajectory/map/controller config hash, 녹화 설정, 종료 원인, row 수,
CSV hash와 metric 요약을 저장한다. GT 전용/SAMPLE 전용으로 사용하지 않는 CSV cell은 빈
문자열로 둔다.

## 오차 정의

각 map-frame Ego 위치를 GT polyline에 투영해 matched index와 station을 구한다.

```text
dx = ego_x - projected_gt_x
dy = ego_y - projected_gt_y

lateral_error      = -sin(gt_yaw) * dx + cos(gt_yaw) * dy
longitudinal_error =  cos(gt_yaw) * dx + sin(gt_yaw) * dy
heading_error      = normalize(ego_yaw - gt_yaw)
velocity_error     = ego_velocity - target_velocity
```

JSON 결과에는 lateral RMS/mean/max, heading mean/max, velocity mean/max와 collision/goal
여부가 기록된다.

## Debug와 history replay

Live와 replay는 같은 출력 계약을 사용한다.

| Topic | Type |
| --- | --- |
| `/trajectory_validator/debug` | `trajectory_validator/TrajectoryDebug` |
| `/trajectory_validator/gt_path` | `nav_msgs/Path` |
| `/trajectory_validator/ego_path` | `nav_msgs/Path` |
| `/trajectory_validator/ego_pose` | `geometry_msgs/PoseStamped` |
| `/trajectory_validator/markers` | `visualization_msgs/MarkerArray` |

`TrajectoryDebug`에는 전체 GT, 현재까지 누적된 Ego trajectory, 현재 Ego/CtrlCmd, matched GT와
오차가 들어간다. CSV의 `GT_POINT`와 `SAMPLE`로 같은 메시지를 복원할 수 있다.

History replay는 요청한 CSV/JSON pair만 읽으며 identity와 CSV SHA-256을 검증한다. 여러
run overlay, 최신 run 자동 선택, 반복 재생은 지원하지 않는다.

실행과 확인 방법은 [README.md](README.md)를 참고한다.
