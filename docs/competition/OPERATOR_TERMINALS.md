# 오퍼레이터 터미널 출력과 오류 대응

처음 실행할 때는 [앞면 터미널 입력표](OPERATOR_START.md)를 먼저 본다.

2026-10-30 기준. 명령별 정상 표시와 우선 확인할 오류를 정리한다.
Network·Sensor 경로와 IP·포트 행 번호는 [네트워크 안내](OPERATOR_NETWORK.md)에 있다.
본선용과 GT를 받는 개발용을 혼동하지 않는다. 이 문서의 진단 명령은 별도 터미널에서 실행한다.

## 실행 순서

각 터미널에서 `source ~/AjouNice2026/docs/common_alias.txt`를 먼저 실행한다.

| 터미널 | 본선 | 팀장님 무GT 시험 |
|---|---|---|
| 1 | `rc` | 동일 |
| 2 | `udp_lan_competition` | `udp_lan_competition_teamlead` |
| 3 | `sensing_lan` | 동일 |
| 4 | `localization_competition` | 동일 |
| 5 | `bevfusion` | 동일 |
| 6 | `tracking` | 동일 |
| 7 | `traffic` | 동일 |
| 8 | `planning` | 동일 |
| 9 | `planviz_competition` | 동일 |
| 10 · 마지막 | `traj_fol_a` | 동일 |

팀장님 GT 비교 실험에서는 2번을 [개발용 UDP 명령](OPERATOR_NETWORK.md)으로,
9번을 `planviz`로 바꾼다. BEVFusion과 `od_gt`를 동시에 실행하지 않는다.
공식 START·P·정지 → 초기 경로 자동 요청과 차량 근처 궤적 확인 → 제어기를 마지막에 실행한다.
Route Status는 표시용이다. 현재 UI에 Route A 요청·Reset 버튼이 있다고 가정하지 않는다.

## 출력 읽는 법

- 요약 로그 앞 `00`~`59`는 현재 시각의 **초**다. 59 다음 00은 정상이다.
- 녹색은 해당 표시의 정상 등급, 노랑은 대기·주의·과거 오류, 빨강은 오류다.
  색 하나로 전체 스택이나 주행 가능 여부를 판정하지 않는다.
- `past3`은 이전 오류가 기록됐다는 뜻이지 지금 같은 오류가 3개 지속된다는 뜻은 아니다.
- `idle`, `log`, `no new logs/status`는 새 원문 로그가 없다는 표시일 수 있다.
  **토픽 Hz·age·내용으로 실제 작동 여부를 확인한다.**
- `V/P/O`는 차량/보행자/장애물 객체 수다. `Hz`는 해당 출력 수신률,
  `ms`는 ROS header 기준 age로 물리적 모델 추론시간이나 네트워크 지연 단독값이 아니다.
- 짧은 화면에서 `~` 등으로 생략돼도 실행을 거부하는 것이 아니다. 원문 로그로 확인한다.

2~7·9번은 compact wrapper, 8번은 판단팀 원문 출력, 10번은 제어팀 전용 표시다.
아래 수치는 출력 형식의 예시이지 보장 성능이나 실행 허용 기준이 아니다.

## 터미널별 정상 출력과 주요 오류

| 터미널 | 주로 볼 표시 | 의미 | 주요 이상과 첫 대응 |
|---|---|---|---|
| 1 rc | `ROS_MASTER_URI`, `started core service` | ROS master 준비 | `Address already in use`: 기존 roscore 확인. 두 번째 master를 띄우거나 전체 kill하지 않음 |
| 2 UDP | `C50 G30 I50 L14.9` | Competition/GPS/IMU/LiDAR 수신 Hz | `waiting Competition/GPS/IMU`, `stale`: MORAI Connect·NIC·IP·포트 확인 |
| 3 TF | `static N/N` | 필요한 센서 장착 TF가 확인됨 | 부족하거나 `TF unavailable`: sensing 중복·설정 경로 확인. 동적 위치까지 정상이라는 뜻은 아님 |
| 4 LOC | `50Hz GPS valid` / `GPS shadow` | 측위 출력과 GPS 유효성 | `waiting global/smooth`, `stale`: Competition·GPS·IMU와 초기화 로그 확인 |
| 5 OD | `V2 P1 O3 10.0Hz 100ms` | 검출 객체 수·출력률·age | `waiting output`, `model timeout`, `GPU failed`, `detector duplicate`: 아래 모델 절차 |
| 6 TRK | `V2 P1 O3 30.0Hz 40ms` | 추적 객체 수·출력률·age | `waiting output`, `TF unavailable`: 검출 입력과 LOC·TF부터 확인. 30Hz만으로 새 검출이 있다는 뜻은 아님 |
| 7 TL | `A GC`, `D RC+RA`, `no mission`, `A far` | 현재 미션의 인지 신호·미관측 상태 | `roi missing`, `no-box`, `CNN unavailable`: 아래 신호등 절차. **인지 신호가 planner GO/STOP 자체는 아님** |
| 8 planning | `VERIFIED route_a/route_b`와 오류 원문 | 경로 요청 거래 검증 | 빨간 궤적·최적화 실패·경로 잠금: 아래 판단 절차. VERIFIED만으로 현재 궤적 위치를 보장하지 않음 |
| 9 VIZ | `map ready pose 50Hz path 10Hz` | 지도·측위·궤적 입력 수신 | `map wait`, `path wait/stale`, Qt/OGRE 오류: 입력과 화면 렌더링을 구분해서 확인 |
| 10 CTRL | `35.20kph 0.15mps2 +2.1°` | 실제 속도·IMU 횡가속도·조향각 | `WAIT`, `controller timeout`, `rx stale`, `pose jump`: 제어 상태·입력 freshness 확인 |

`GPS shadow`는 음영구간에서 예상할 수 있다. 음영 밖에서도 지속되거나 측위가 끊기면 조사한다.
Collision은 충돌 때만 나올 수 있어 무충돌 상태에서 무출력이라는 이유로 고장으로 판단하지 않는다.
모델 준비 중에는 요약 화면이 `waiting output`일 수 있다. 원문의 `Model waiting/ready`와 함께 본다.
준비 대기 300초·선택적 무제한과 해당 진행 문구는 이번 로컬 수정이며 아직 PR 미반영이다.
팀장님 설치본에 반영됐다고 가정하지 않는다. 기존 배포본은 2초 간격 최대60회 확인이다.

## 1 네트워크와 센서 오류

```bash
ip -br -4 addr
rosparam get /morai_ego_udp_receiver/network_config
rosparam get /morai_sensor_udp_receiver/sensor_config
timeout 5s rostopic hz /Competition_topic
timeout 5s rostopic hz /sensor/gps
timeout 5s rostopic hz /sensor/imu
timeout 5s rostopic hz /sensor/lidar1
```

- `Cannot assign requested address`: ROS PC에 JSON의 destinationIP가 있는지 확인한다.
  본선은 192.168.0.10, 팀장님 개발망은 10.42.0.1이다. 현장에서 임의 IP로 고치지 않는다.
- `Address already in use`: 아래에서 소유 프로세스를 찾는다. 자기 터미널의 중복 수신기만
  정리한다. `SO_REUSEADDR` 때문에 중복이 반드시 오류로 검출되는 것은 아니다.
- 카메라는 보이는데 C/G/I가 안 나오면 Sensor 연결과 Ego Network 연결을 따로 확인한다.
- L4Hz처럼 낮으면 MORAI·CPU 부하를 확인한다. 이를 GNSS 지연 보상값으로 해결하려 하지 않는다.

```bash
ss -lunp | rg '9092|9093|9094|9099|9291|9293|9295|9297|9301|9303|2368'
```

## 2 측위와 TF 오류

```bash
timeout 3s rostopic echo -n 1 /localization/kinematic_state/header
timeout 3s rostopic echo -n 1 /localization/control_kinematic_state/header
timeout 3s rosrun tf tf_echo map base_link
```

`Refusing official START initialization: GNSS is ... m away`는 START에서 시작하지 않았거나
좌표/입력 문제다. 다른 위치를 START로 강제 초기화하지 않는다. GNSS·heading 시각 불일치도
초기화를 지연시킬 수 있다. 노드가 살아 있거나 odom이 나온다는 사실만으로 초기화 성공은 아니다.
`Published GNSS+IMU initial pose from official_start`를 원문에서 확인한다.
초기화·측위 문제를 GT 측위로 몰래 대체하지 않는다. 리스폰 때 START 초기화를 반복하지 않는다.

## 3 모델 기동과 검출 오류

```bash
nvidia-smi
docker ps -a --filter label=com.docker.compose.service=inference --format '{{.Names}}  {{.Status}}'
rostopic info /perception/object_recognition/detection/objects
```

확인한 컨테이너 이름으로 `docker logs --tail 80 <이름>`을 실행한다.

- `detector duplicate`: publisher 목록에서 기존 정상 BEVFusion·od_gt·bag 재생 여부를 확인한다.
  정상 detector가 이미 실행 중이면 추가 실행하지 않는다. 무작정 `rosnode kill -a` 하지 않는다.
- `model timeout`: 모델 서버 로딩/워밍업인지, 모델·센서 일치 검사 실패인지 원문을 구분한다.
  다른 프로필 서버가 남아 있다고 자동으로 종료하지 않는다.
- checkpoint 누락·SHA mismatch: 배포 파일을 확인한다. 검사 해제·임의 모델 대체를 하지 않는다.
- CUDA OOM/illegal memory: 증거를 보존하고 중복 GPU 작업을 확인한다. 주행 중 즉석 서버 재시작 금지.
- `sync`: 4카메라·LiDAR 시각 조합 실패다. 모든 센서 수신과 로컬 CPU 부하를 확인한다.
  새 검출이 없는데 tracker 출력만 있는 상태를 정상 인지로 보지 않는다.

`bevfusion --safe`는 같은 E12의 추가 가속만 끄는 선택지다. 출발 전 합의·시험한 경우에만 사용하며
실행 중 기존 detector 옆에 두 번째로 켜지 않는다.

## 4 신호등 오류

```bash
timeout 3s rostopic echo -n 1 /traffic_light_node/debug_selection
timeout 3s rostopic echo -n 1 /traffic_light_node/debug_classifier
timeout 3s rostopic echo -n 1 /traffic_light_node/debug_policy_reason
```

`far/no mission`은 멀리 있거나 해당 미션이 없는 경우 정상일 수 있다. 정지선 통과 후 화면 밖으로
나간 신호도 미관측일 수 있다. **화면 안에 충분히 큰 미션 신호가 있는데 연속 no-box/roi missing**이면
카메라 → 측위/경로 선택 → 지도 ROI·미션 매칭 순서로 조사한다. roi missing만으로 YOLO 고장이라고
단정하지 않는다. `mission updating`은 미션 선택과 영상 결과가 교체 중이라는 표시다.
`CNN unavailable`은 모델/환경 오류이며 과거 신호가 보였다는 이유로 정상으로 취급하지 않는다.
진단 목적으로 GO를 강제하거나 신호등 인지를 끄지 않는다.

## 5 플래닝 실패와 경로 잠금

```bash
timeout 3s rostopic echo -n 1 /competition_route_requester/request_status
timeout 3s rostopic echo -n 1 /planning/route_state
timeout 3s rostopic echo -n 1 /planning/trajectory/header
```

- 빨간 궤적·`Invalid Trajectory`는 원문 사유를 확인한다. 단발 실패는 다음 계산에 회복될 수 있다.
  정당한 신호/장애물 정지와 최적화 실패를 구분한다. 메시지 header가 신선해도 차량에서 먼
  과거 궤적일 수 있으므로 RViz에서 현재 차량과 연결되는지도 확인한다.
- `RECONCILIATION_REQUIRED`: 경로 요청 결과가 불확실하여 추가 요청을 잠근 상태다.
  현재 route UUID·실제 경로를 확인한다. 안전한 자동 해제/Reset 명령은 아직 제공하지 않는다.
  노드 하나만 재시작하면 초기 Route A를 다시 요청할 수 있어 중간 코스에서 무작정 재시작하지 않는다.
- 원격 `44be5053`에는 정지 최적화 실패의 recovery mode가 포함됐다. 해당 빌드에서만
  `/planning/recovery/status`를 확인한다. 모든 실패를 복구하거나 장애물을 밀어내는 기능은 아니다.
  `Trajectory` 메시지 변경이 있어 서로 다른 버전의 planning/base/플러그인 혼용을 피한다.
- 리스폰은 운영측 승인 뒤 정지 상태에서 한다. 가까운 CP1 복귀는 로컬1회 복구됐지만
  모든 CP·즉시 재출발 보장은 아니다. 차량 근처 새 궤적과 측위 정합 전 출발하지 않는다.

## 6 제어기와 화면 오류

```bash
timeout 3s rostopic echo -n 1 /control/terminal_state
timeout 3s rostopic echo -n 1 /ctrl_cmd
```

CTRL의 `mps2`는 표시상 IMU 횡가속도이며 제동거리 설정 감속도가 아니다.
`age(T/S)`가 원문에 보이면 궤적/상태의 수신 후 경과시간이다.
`rx stale`·시각 오류면 8번 궤적과 4번 측위부터 확인한다. 보호 제동을 해제하지 않는다.
`controller timeout`은 전용 표시가 제어 상태를 못 받는다는 뜻으로 제어 명령 정상까지 보장하지 않는다.
`pose jump` 보호는 위치 급변 대응이며 지속되면 측위를 조사한다.
RViz `Qt/OGRE/display unavailable`은 화면 문제일 수 있다. LOC·planning 노드가 살아 있는지
별도로 확인하고, 화면만 재시작할 상황과 전체 스택 문제를 구분한다.

## 원문 로그와 금지할 응급 조치

공통 기동 오류도 구분한다.

| 오류 원문 | 먼저 확인 | 대응 |
|---|---|---|
| `command not found`, package not found | alias source와 `rospack find <패키지>` | 올바른 checkout의 환경 재source. 실행 중 다른 checkout을 섞지 않음 |
| `unused args`, 새 인자를 받지 못함 | 부모 launch·서브모듈·설치본 버전 | 버전 묶음을 맞춰 재설치할 문제. 인자를 임의 삭제해서 새 정책이 적용된 척하지 않음 |
| `md5sum` mismatch, message datatype mismatch | 새 `Trajectory` 포함 메시지 빌드·bag 버전 | planning/base/플러그인의 동일 메시지 빌드 확인. 연결 검사를 끄지 않음 |
| `ImportError`, `ModuleNotFoundError`, native `.so` 로딩 오류 | 실제 Python과 설치 overlay | 신호등 GPU 환경·시스템 helper 환경을 구분. 현장에서 임의 pip upgrade 금지 |
| `process has died`, `Traceback`, `FATAL` | 죽은 노드 이름·마지막 원문 | 해당 노드·원문 보존 후 원인별 대응. 모든 노드가 죽었다고 가정하지 않음 |

```bash
field_log_list
df -h ~/AjouNice2026
```

wrapper 원문은 해당 명령이 안내한 `*_console/*_raw.log`, 요약은 `*_status.jsonl`에 있다.
`field_log_list`는 현재 터미널이 공유하는 세션만 보여 주며 수동 실행/다른 세션 전체를 모으지는 않는다.
8번 planning과 10번 제어기는 wrapper 자동 기록 대상이 아니다. ROS 로그 `~/.ros/log/`와
기록 bag을 함께 보존한다. 영상만으로 입력·판단·제어 원인을 모두 재구성할 수는 없다.

`origin traffic` 같은 원문 모드는 **기존 해당 노드를 정상 종료한 뒤 새로 실행할 때** 사용한다.
실행 중인 원문을 보기 위해 같은 노드를 하나 더 시작하지 않는다.
`raw write failed`·disk full이면 기록을 보존할 수 없다는 뜻이다. 임의 원본 데이터 삭제 대신
합의한 정리 대상을 사용한다. 로그 폭·표시 문제 때문에 알고리즘을 재시작하지 않는다.

**Ctrl+C는 차량 정지 명령이 아니다.** 재시작·설정 변경·프로세스 종료는 차량 실제 정지 확인과
운영측 협의 후 수행한다. 전체 kill, 보호 제동 해제, 잘못된 입력 강제 허용, 중간 코스의
무조건 Route A 재요청은 대응 절차로 사용하지 않는다.
