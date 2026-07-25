# Trajectory Validator 설계 요약

## 목적과 모드

`trajectory_validator`는 follower 추종 결과를 저장하고 이전 run 하나를 다시 publish한다.
trajectory와 제어 명령은 생성하거나 수정하지 않는다.

| 모드 | 진입점 | 역할 |
| --- | --- | --- |
| Live record | `record.launch` | 현재 주행을 CSV/JSON으로 저장하고 시각화 토픽 발행 |
| History replay | `history.launch mode:=replay` | 선택한 run을 시간순으로 복원 |
| Final path | `history.launch mode:=final` | 선택한 run의 전체 경로를 즉시 정적으로 표시 |
| Compare | `history.launch mode:=compare` | 선택한 run을 compare 전용 토픽에 정적으로 표시 |

RViz와 rqt는 `viewer` 패키지에서만 실행한다. `trajectory_validator.launch`와 `publish_history.py`는 기존
호출부와의 호환성을 위해 남겨 둔다.

기록 설정은 별도 YAML로 복제하지 않는다. `record.launch`가 실행 중인
`/trajectory_publisher`, `/purepursuit`, `/longi_pi` 파라미터를 캡처한다. 과거 run 선택만
`history_run.yaml`로 관리한다. 비교 기준은 `compare_run.yaml`로 따로 관리하며 compare와
PlotJuggler가 공유한다.

## Live record

입력:

| Topic | Type |
| --- | --- |
| `/control/trajectory` | `autoware_planning_msgs/Trajectory` |
| `/localization/kinematic_state` | `nav_msgs/Odometry` |
| `/Ego_topic` | `morai_msgs/EgoVehicleStatus` (차량 고유 telemetry) |
| `/ctrl_cmd` | `morai_msgs/CtrlCmd` |
| `/CollisionData` | `morai_msgs/CollisionData` (선택) |

첫 유효 trajectory 전체를 GT로 한 번 저장하고 Ego 상태를 기본 50 Hz로 sampling한다. 위치,
자세, 속도는 `map -> base_link` 공통 kinematic state에서 읽고 가속도·휠각 등 MORAI 전용 값만
`/Ego_topic`에서 읽는다. trajectory와 state frame이 다르면 sample을 유효 처리하지 않는다.
노드 실행 직후에는 armed 상태이며 trajectory, kinematic state, Ego 상태, CtrlCmd가 모두
fresh하고 timestamp 동기 조건을 만족한 최초 시점부터 sample을 기록하고 경과시간을 0으로 잡는다.
Follower가 아직 실행되지 않아 `/purepursuit_autoware`, `/longi_pi_autoware`, `/ctrl_cmd`가 없어도 노드는 종료하지
않는다. 최초 유효 입력 직전에 runtime parameter를 캡처하므로 Validator를 Follower보다 먼저
실행할 수 있다.
`control_pair_valid`는 입력이 fresh하고 상태-명령 수신 차이가 허용 범위 안일 때 true다.
통계는 `control_pair_valid=true`인 sample만 사용한다.

기본 설정에서는 collision을 기록만 하고 run을 계속 진행한다. `stop_on_collision=true`를
설정한 경우에만 collision, goal 도달 또는 ROS shutdown으로 run을 종료한다. 종료 시 CSV를 닫은 뒤 JSON을
`status=complete`로 원자적으로 갱신한다.
MORAI의 충돌 없음 placeholder(`ObjectStatus.type=-1`)는 collision으로 계산하지 않는다.

## 저장 구조

```text
history/<traj_sample>/<MMDD_HHMMSS_pp_autoware>/tracking_samples.csv
history/<traj_sample>/<MMDD_HHMMSS_pp_autoware>/metadata.json
```

- `traj_sample`: 입력 trajectory CSV의 확장자를 제외한 basename
- 동일한 run ID가 있으면 `_01`, `_02` 순으로 새 이름 사용
- 이름은 `[A-Za-z0-9_-]`만 허용

schema v5 파일:

- `tracking_samples.csv`: station, Ego pose/속도/yaw rate, 목표 속도, LD, 경로·목표 곡률,
  횡/헤딩 오차, 명령·실제 조향각, 유효성 및 collision의 21개 열
- `metadata.json`: 원본 및 trajectory별 단일 reference 경로/hash, follower runtime parameter, 실행 설정,
  결과 통계, row 수와 compact CSV의 SHA-256

기존 schema v3/v4 기록도 replay/final path로 읽을 수 있다.

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

History replay는 요청한 run 디렉터리의 세 파일만 읽으며 identity와 각 CSV SHA-256을 검증한다. 여러
run overlay, 최신 run 자동 선택, 반복 재생은 지원하지 않는다.

Final path 모드는 같은 무결성 검사를 수행한 뒤 모든 SAMPLE을 하나의 Ego Path로 만들어 latch
발행한다. 시간축과 rqt debug가 필요하면 Replay를, 공간적인 최종 경로만 보면 Final path를 쓴다.

실행과 확인 방법은 [README.md](README.md)를 참고한다.
