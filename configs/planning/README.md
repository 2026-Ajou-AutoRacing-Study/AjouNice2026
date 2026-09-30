# 플래닝·제어 YAML 실행 경로

플래닝/제어 실행 파일과 라이브러리는 install 공간을 사용하지만, YAML은
`AJOUNICE_ROOT` 아래 워크스페이스 소스를 직접 읽는다. `$(find package)/config`
형식은 install 복사본을 선택할 수 있으므로 운영 launch의 YAML 기본 경로로 사용하지 않는다.

## 파라미터 수정 순서

1. 아래 실제 로딩 대상 소스 YAML을 수정하고 저장한다.
2. 해당 노드 또는 해당 실행 스택을 재시작한다.
3. `rosparam get /노드/파라미터`로 로딩 값을 확인한다.

YAML만 변경하면 컴파일/설치가 필요 없다. 실행 중 자동 재로딩은 제공하지 않는다.
launch 자체 또는 C++/Python 실행 코드 변경의 설치/빌드 절차는 별개다.
`AJOUNICE_ROOT`는 사용하는 체크아웃 루트여야 한다.

## 주요 운영 설정

| 대상 | 소스 YAML |
|---|---|
| 차량 플래닝 차체 | `configs/planning/vehicle_info.yaml` |
| Pure Pursuit | `base_ws/src/control/trajectory_follower/config/purepursuit_autoware.yaml` |
| 종방향 PI | `base_ws/src/control/trajectory_follower/config/longi_pi_autoware.yaml` |
| 제어 공통 | `base_ws/src/control/trajectory_follower/config/trajectory_follower_autoware.yaml` |
| SOA 등 BPP 모듈 | `planning_ws/src/autoware_launch/config/ros1/planning/behavior_path_modules.yaml` |
| BVP 통합 모듈 설정 | `planning_ws/src/autoware_launch/config/ros1/planning/behavior_velocity_modules.yaml` |
| PO | `planning_ws/src/autoware_launch/config/ros1/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml` |
| Velocity smoother | `planning_ws/src/autoware_launch/config/ros1/planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| 라우트 요청 정책 | `base_ws/src/planning_addons/competition_route_requester/config/competition_route_requester.param.yaml` |

`planning` 명령의 roundabout 파라미터는 BVP 통합 모듈 설정의 `roundabout:` 항목에서만 수정한다.
roundabout 관련 ROS 1 테스트도 이 파일을 읽는다. 모듈의 ROS 2 형식 설정은 별도 환경용이다.
개별 패키지의 독립 실행용 YAML과 통합 실행용 YAML은 용도가 다를 수 있다.
이 변경은 운영 launch가 선택하는 파일을 소스에서 읽도록 만든다. 여러 YAML이
겹치는 키를 정의하거나 launch가 명시적으로 값을 덮어쓰는 경우 기존 우선순위를 유지한다.
아래 검사 결과가 실제 로딩 파일 목록이다.

## 실행 없이 검사

워크스페이스 ROS 환경을 source한 뒤:

```bash
python3 scripts/runtime/check_source_yaml_paths.py \
  --json planner_logs/analysis/source_yaml_paths/resolved.json
```

설치된 planning, morai_planning_stack, pp_pi_autoware launch를 해석해 YAML 경로를 확인한다.
노드를 시작하거나 실행 중 ROS 파라미터를 변경하지 않는다. install/devel YAML 복사본,
누락 파일 또는 상대 파일 경로가 발견되면 실패한다. 통합 스택의 추적·예측·신호 프로필도 검사한다.
