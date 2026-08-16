# MORAI 대회판과 원본 Autoware 0.51 검증

이 디렉터리는 MORAI `25.S4.MolitComp03` 대회판을 원본 Autoware
Universe 0.51.0에 연결한 과정과 재현 도구를 보관한다.

핵심 원칙은 Autoware 플래닝·제어 알고리즘을 수정하지 않는 것이다.
`morai_autoware_interface`는 서로 다른 메시지 계약을 변환하는 외부 ROS 2
패키지이고, `morai_rosbridge_compat.py`는 MORAI 대회판의 ROS1 형태 JSON을
ROS 2 rosbridge 경계에서 정규화한다.

## 현재 검증 상태

| 항목 | 상태 | 근거 |
|---|---|---|
| Ubuntu 22.04 + ROS 2 Humble | PASS | 개발 환경 설치 완료 |
| 고정된 Autoware 소스 빌드 | PASS | 핵심 패키지 조회 및 전체 빌드 완료 |
| Planning Simulator | PASS | K-City 지도에서 경로·궤적·제어 출력 확인 |
| MORAI ROS 2 입력 | PASS | `/Ego_topic` 약 50 Hz 실측 |
| 메시지 변환 어댑터 | PASS | 빌드, 단위 테스트, 안전 검증 통과 |
| MORAI 연속 제어 | PASS | 12 m 시험에서 337개 연속 위치 표본, 11.72 m 이동 |
| K-City 전체 경로 생성 | PARTIAL | 0.51의 순환 경로 제한을 확인하고 2구간 중 1구간 생성 성공 |
| 대회 코스 전체 물리 주행 | **미완료** | 시나리오 재초기화 후 MORAI 네트워크 재연결 직전 중단 |

따라서 이 자료는 연결과 짧은 폐루프 주행이 검증된 상태이며, 약 1.94 km
전체 코스 완주 결과를 의미하지 않는다.

자세한 실측값과 남은 작업은
[`VALIDATION_REPORT_2026-08-17.md`](VALIDATION_REPORT_2026-08-17.md)에 있다.

## 고정 버전

| 구성 | 버전 또는 커밋 |
|---|---|
| 운영체제 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Autoware manifest | `4847819760656247343dfb995ea8e151fc3d9b00` |
| autoware_universe | `0.51.0` / `d4d260983d357e1b2b34291d91933f9f4b53bf94` |
| autoware_core | `1.8.0` / `16045061a9c10da468b60e190b8fab02110fa501` |
| autoware_launch | `0.51.0` / `3ba12ba6a47abca105e755e6d21cf66a0cdf743f` |
| MORAI | `25.S4.MolitComp03` |
| 지도 | K-City Lanelet2 v60 |

Autoware 작업 경로는 `/home/ajou/autoware-0.51`, 지도 경로는
`/home/ajou/autoware_data/maps/kcity-v60`을 기준으로 기록했다.

## 저장 내용

```text
reference/autoware_original_validation/
├── README.md
├── VALIDATION_REPORT_2026-08-17.md
├── adapter/morai_autoware_interface/    # 독립 ROS 2 변환 패키지
├── scripts/
│   ├── reset_morai_scenario.py          # 대회 시나리오 초기화
│   └── set_kcity_route_phase.py         # 2구간 Lanelet 경로 설정
└── tools/morai_rosbridge_compat.py      # websocket 경계 호환 처리
```

빌드 산출물, Python 캐시, 지도 원본, 토큰과 인증 정보는 포함하지 않는다.

## 메시지 경계

MORAI에서 실제 확인한 주요 인터페이스는 다음과 같다.

| 방향 | MORAI 인터페이스 | Autoware 인터페이스 |
|---|---|---|
| 입력 | `/Ego_topic` (`morai_msgs/msg/EgoVehicleStatus`) | 차량 상태, `/localization/kinematic_state`, TF |
| 입력 | `/Object_topic` (`morai_msgs/msg/ObjectStatusList`) | `/perception/object_recognition/objects` |
| 출력 | `/ctrl_cmd` (`morai_msgs/msg/CtrlCmd`) | `/control/command/control_cmd`에서 변환 |
| 양방향 | `/Service_MoraiEventCmd` | 제어 모드, 기어, 방향지시등, 비상등 |

어댑터 기본 단위는 다음과 같다.

- MORAI ego 속도: m/s
- MORAI object 속도: km/h에서 m/s로 변환
- MORAI `wheel_angle`: degree에서 radian으로 변환
- Autoware 목표 속도: m/s에서 MORAI km/h로 변환
- 위치·방향: MORAI map 좌표를 `map -> base_link`로 게시

대회판 rosbridge JSON에는 ROS 2 메시지에 없는 `Header.seq`와 추가 필드가
포함된다. 제어 출력은 ROS1 이름인 `longlCmdType`, `front_steer`,
`rear_steer`를 기대한다. 호환 도구는 이 차이만 websocket 경계에서 처리한다.

## 설치

어댑터를 고정된 Autoware 소스 트리에 복사한다.

```bash
cp -a \
  reference/autoware_original_validation/adapter/morai_autoware_interface \
  /home/ajou/autoware-0.51/src/vehicle/external/

cd /home/ajou/autoware-0.51
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select morai_autoware_interface
source install/setup.bash
```

호환 rosbridge는 원하는 위치에 복사하거나 저장소 경로에서 직접 실행한다.

## MORAI 네트워크 설정

MORAI의 두 네트워크 탭을 모두 다음 값으로 연결한다.

| 탭 | 프로토콜 | IP | 포트 |
|---|---|---|---|
| Cmd Control | ROS | `127.0.0.1` | `9090` |
| Publisher, Subscriber, Service | ROS | `127.0.0.1` | `9090` |

Cmd Control은 `morai_msgs/CtrlCmd`, `/ctrl_cmd`로 설정한다. 시나리오를
다시 불러오면 연결이 끊기므로 두 탭을 다시 연결해야 한다.

## 실행 순서

터미널 1에서 호환 rosbridge를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/ajou/autoware-0.51/install/setup.bash
python3 reference/autoware_original_validation/tools/morai_rosbridge_compat.py \
  --ros-args -p port:=9090 -p max_message_size:=10000000
```

터미널 2에서 어댑터를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/ajou/autoware-0.51/install/setup.bash
ros2 launch morai_autoware_interface morai_autoware_interface.launch.py
```

터미널 3에서 원본 Autoware를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/ajou/autoware-0.51/install/setup.bash

ros2 launch autoware_launch autoware.launch.xml \
  map_path:=/home/ajou/autoware_data/maps/kcity-v60 \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  launch_sensing:=false \
  launch_localization:=false \
  launch_perception:=false \
  launch_vehicle_interface:=false \
  use_sim_time:=false \
  system_run_mode:=planning_simulation \
  launch_system_monitor:=false \
  launch_dummy_diag_publisher:=true \
  is_simulation:=true \
  rviz:=true
```

MORAI 대회판은 `/clock`을 광고하지만 이번 시험에서는 메시지를 발행하지
않았으므로 `use_sim_time:=false`가 필요했다.

## 안전 검증

실제 `/ctrl_cmd`를 사용하지 않는 검증은 다음과 같이 실행한다.

```bash
cd /home/ajou/autoware-0.51
./src/vehicle/external/morai_autoware_interface/scripts/verify_safe.sh
```

검증 launch는 제어 출력을 `/verification/morai_ctrl_cmd`로 remap한다. 실제
MORAI 차량에 명령을 보내지 않으면서 메시지 타입, 주기, 변환값과 0.5초
명령 timeout 브레이크를 확인한다.

## 대회 경로 설정

Autoware 0.51 RouteHandler는 시작 직전 차로로 되돌아오는 순환 경로를 한
번에 받지 않는다. 전체 41개 차로는 다음 두 구간으로 설정한다.

```bash
python3 reference/autoware_original_validation/scripts/set_kcity_route_phase.py 1
# 1구간 도착 후
python3 reference/autoware_original_validation/scripts/set_kcity_route_phase.py 2
```

1구간은 `200889`에서 `200445`, 2구간은 겹침 차로 `200445`에서
`200892`까지다. 구간 설정 도구는 자율주행 시작 서비스는 호출하지 않는다.

## 주의 사항

- 어댑터는 Autoware 원본 알고리즘을 수정하지 않는다.
- `Object_topic` 변환은 MORAI ground truth를 사용하는 방식이다. 원본
  센서 인지 성능을 비교할 때는 `publish.objects:=false`로 분리한다.
- `command.steering_sign`, `report.steering_sign`, 최대 속도는 실제 차량 모델과
  대회 규정에 맞춰 트랙에서 확인해야 한다.
- 전체 코스 완주, 신호등 입력, 동적 객체 대응은 아직 최종 판정하지 않았다.
