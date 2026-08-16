# 원본 Autoware–MORAI 검증 기록 (2026-08-17)

## 목적

팀에서 포팅할 플래닝과 원본 Autoware의 동작을 같은 MORAI 대회 환경에서
비교할 수 있도록 원본 기준선을 만든다. 이번 기준선은 Universe 0.51.0이며
변환 계층 이외의 Autoware 소스는 수정하지 않는다.

## 설치와 빌드

- 개발 환경 Ansible: 최종 `ok=245`, `failed=0`
- CUDA 드라이버 610 설치는 기존 NVIDIA 580 계열과 충돌했으나, 기존
  드라이버를 유지하는 방식으로 개발 환경 설치를 완료했다.
- `autoware_trajectory_optimizer`를 재빌드했고 경고만 발생했다.
- 다음 패키지 조회에 성공했다.
  - `autoware_launch`
  - `autoware_trajectory_optimizer`
  - `autoware_simple_planning_simulator`
- Planning Simulator에서 K-City 지도, localization, route, planning,
  autonomous control 상태를 화면과 토픽으로 확인했다.

## MORAI 인터페이스 실측

MORAI 네트워크를 ROS, `127.0.0.1:9090`으로 연결한 뒤 다음 토픽을 확인했다.

```text
/CollisionData   morai_msgs/msg/CollisionData
/Competition_topic morai_msgs/msg/EgoVehicleStatus
/Ego_topic       morai_msgs/msg/EgoVehicleStatus
/Object_topic    morai_msgs/msg/ObjectStatusList
/SaveSensorData  morai_msgs/msg/SaveSensorData
/ScenarioLoad    morai_msgs/msg/ScenarioLoad
/SetTrafficLight morai_msgs/msg/SetTrafficLight
/ctrl_cmd        morai_msgs/msg/CtrlCmd
/tf              tf2_msgs/msg/TFMessage
```

`/Ego_topic` 실측 예시는 다음과 같다.

```text
frame_id: MoraiInfo
position: [-104.0405, -378.6016, 28.8885]
velocity: [4.3033, -0.0045, 0.0233]
heading: 60.9638
wheel_angle: -0.0752
average rate: approximately 49.5 Hz
```

처음 토픽 이름만 보이고 데이터가 없던 원인은 MORAI 시뮬레이션/네트워크
연결 상태였으며, 재연결 뒤 약 50 Hz 수신을 확인했다.

## 호환 문제와 해결 범위

1. MORAI 대회판의 rosbridge 입력 JSON은 ROS1 `Header.seq`와 공개 ROS 2
   `morai_msgs`에 없는 필드를 포함했다.
2. 대회판의 `CtrlCmd` websocket 계약은 ROS1 필드 이름
   `longlCmdType/front_steer/rear_steer`를 기대했다.
3. Autoware와 MORAI는 차량 상태·제어 명령의 메시지 타입과 단위가 다르다.

해결은 두 개의 외부 계층으로 제한했다.

- `morai_rosbridge_compat.py`: 알 수 없는 입력 필드 제거와 출력 alias 추가
- `morai_autoware_interface`: 차량 상태, 제어, 기어, lamp, 객체 메시지 변환

`autoware_universe`, `autoware_core`, `autoware_launch` 소스에는 변경이 없다.

## 어댑터 검증

- `/Ego_topic`에서 velocity, steering, acceleration, odometry와
  `map -> base_link` TF 생성
- `/Object_topic`에서 `PredictedObjects` 생성
- `/ctrl_cmd` BEST_EFFORT QoS 연결 확인
- dry-run: Autoware `2.0 m/s`, `0.5 m/s^2`, `0.1 rad`가 MORAI
  `7.2 km/h`, `0.5 m/s^2`, `0.1 rad`로 변환
- 0.5초 제어 명령 timeout 시 full brake watchdog 확인
- 변환 단위 테스트 통과

## 짧은 폐루프 주행

전체 코스에 앞서 약 12 m 경로로 실제 제어를 확인했다.

- 연속 위치 표본: 337개
- 총 이동 거리: 11.72 m
- 표본 간 최대 이동: 0.116 m
- 충돌: 0
- 도착 후 STOP/PARK 전환

이 결과는 연속 제어 연결의 증거이며 전체 코스 완주의 증거는 아니다.

## K-City 전체 코스 조사

기존 competition route publisher에서 다음 41개 차로 순서를 확인했다.

```text
200889, 200392, 200178, 200337, 200287, 200348, 200346, 200291,
200292, 200305, 200841, 200950, 200310, 200308, 200258, 200261,
200315, 200445, 200311, 200968, 201095, 201098, 201096, 201087,
201086, 200101, 200128, 200102, 200134, 200360, 201399, 200088,
201401, 200183, 200037, 200471, 201071, 200182, 200361, 200181,
200892
```

K-City v60 지도에서 모든 ID가 존재함을 확인했고, centerline 합산 길이는
약 1,941 m였다. 마지막 `200892`는 첫 `200889` 직전으로 연결되어 전체
목록이 순환 경로가 된다.

Autoware 0.51 RouteHandler에 41개를 한 번에 전달하면 다음 이유로 거부됐다.

```text
Loop detected within route! Currently, no loop is allowed for route!
```

따라서 1구간 `200889..200445`와 2구간 `200445..200892`로 분할했다.

## 1구간 플래닝 결과

1구간 18개 segment를 `/api/routing/set_route`로 전달한 결과:

- route state: `2` (SET)
- operation mode: 자율주행 모드 사용 가능
- `/planning/trajectory`: 약 10 Hz
- intersection processing time: 약 1.711 ms
- 경로 시작 lane: `200889`
- 경로 끝 lane: `200445`

초기 전체 경로 조사에서는 Behavior Path Planner의 기본 전방 길이 300 m로
인해 intersection 계산이 약 543 ms, trajectory가 약 1 Hz까지 떨어졌다.
진단용으로 새 프로세스 시작 전에 다음 값을 일시적으로 적용했다.

```text
Behavior Path Planner forward_path_length: 300 -> 100
Intersection attention length: 200 -> 80
Intersection path interpolation interval: 0.1 -> 0.5
```

노드가 값을 읽은 후 원본 파일은 즉시 복구했고 Autoware 작업 트리의 diff가
없음을 확인했다. 이 값은 성능 원인 규명을 위한 실험값이며, 정식 파라미터
결정이나 원본 변경으로 간주하면 안 된다. 재현용 parameter overlay는 후속
검증에서 별도로 만들어야 한다.

## 마지막 상태와 다음 절차

`2026_molit_comp_sample_scene`을 출발점으로 다시 불러왔다. 시나리오 로드는
성공했고 예상대로 rosbridge client count가 12에서 1로 감소했다. 여기서
MORAI 네트워크 두 탭을 다시 연결하기 전에 작업을 중단했다.

다음 검증자는 아래 순서로 이어간다.

1. Cmd Control과 Publisher/Subscriber/Service를 모두 ROS
   `127.0.0.1:9090`으로 재연결한다.
2. `/Ego_topic` 약 50 Hz와 `/client_count`를 확인한다.
3. 1구간 route state, trajectory 약 10 Hz, autonomous availability를 확인한다.
4. MORAI 네트워크 설정 창을 닫고 1구간 자율주행을 시작한다.
5. 이동 거리, 최대 위치 점프, 정지 원인, 충돌 수를 지속 기록한다.
6. `200445` 도착 후 2구간을 설정하고 다시 autonomous로 전환한다.
7. `200892` 도착까지 실제 총거리와 충돌/개입 여부를 기록한 뒤에만 전체
   코스 PASS로 판정한다.

신호등 상태 변환과 센서 기반 인지 비교는 아직 완료하지 않았다. 현재
`Object_topic`은 MORAI ground truth를 Autoware 객체로 변환하므로 원본 인지
스택과의 비교 시험에서는 이 입력을 분리해야 한다.
