# MORAI MR BEVFusion tracking-only 수동 주행 녹화
> **CURRENT 운영 runbook.** 대회용 MR에서 사람이 직접 운전하며 BEVFusion 검출과
> tracking ID 유지 상태를 경량 rosbag으로 기록할 때 사용한다. Planning, control,
> traffic-light, RViz는 실행하지 않는다. 전체 자율주행은
> [`morai_full_stack_by_terminal.md`](morai_full_stack_by_terminal.md)를 사용한다.

## 이 구성에서 실행하는 것

```text
MR sensor/Ego network
  -> udp                         # UDP packet -> ROS topic
  -> sensing                     # base_link -> sensor static TF
  -> localization               # MR /Competition_topic
  -> bevfusion                  # E25 detection
  -> tracking                   # EKF + stationary policy + Map-CV
  -> lightweight rosbag
```

실행하지 않는 항목은 `traffic`, `planning`, `traj_fol_a`, `planviz`다. MORAI 차량은
사람이 직접 조작한다.

`udp`와 `sensing`은 중복이 아니다.

- `udp`: camera/GPS/IMU/Ego/Object UDP 수신과 LiDAR 변환.
- `sensing`: sensor JSON의 extrinsic으로 LiDAR/camera/GPS/IMU static TF 발행.

둘 중 `sensing`을 생략하면 BEVFusion이 요구하는 sensor TF가 사라진다. 둘을 동시에
실행해도 UDP port를 중복 점유하지 않는다.

## MORAI 설정

대회용 MR의 local loopback 설정은 다음 두 파일을 MORAI에서 직접 로드한다.

```text
Sensor      : configs/sensors/Sensor_competition_local.json
Ego Network : configs/network/ego_network_competition_local.json
```

ROS에서 제어하지 않으므로 UDP gateway는 `enable_control:=false`로 실행한다.

## 모든 새 터미널의 준비

각 터미널에서 아래 두 줄을 먼저 실행한다.

```bash
source ~/.bashrc
ajou
```

`ajou`는 repository root로 이동하고 `AJOUNICE_ROOT`, AW149 overlay 및
`docs/common_alias.txt`를 준비한다.

## 터미널별 실행 순서

### Terminal 1 — ROS master

```bash
source ~/.bashrc
ajou
rc
```

### Terminal 2 — UDP sensor/Ego/Object gateway

```bash
source ~/.bashrc
ajou
udp enable_control:=false
```

### Terminal 3 — sensor static TF only

```bash
source ~/.bashrc
ajou

sensing \
  sensor_config:="$AJOUNICE_ROOT/configs/sensors/Sensor_competition_local.json" \
  enable_vehicle_model:=false \
  start_rviz:=false
```

### Terminal 4 — MR localization

```bash
source ~/.bashrc
ajou
localization
```

`MR`은 `/Competition_topic`을 제공하므로 `localization`이 맞다. `localization_mrp`는
2026 개인용 MRP에서 `/Ego_topic`을 사용할 때만 실행한다.

### Terminal 5 — BEVFusion E25

```bash
source ~/.bashrc
ajou
bevfusion
```

Docker model ready와 ROS bridge 시작을 확인한 뒤 다음 단계로 간다.

### Terminal 6 — production tracking + prediction

```bash
source ~/.bashrc
ajou
tracking
```

이 alias는 외부 BEVFusion detector를 사용하므로 detector를 중복 실행하지 않으며,
BEVFusion용 tracking age `0.5 s`와 Map-CV prediction을 적용한다.

### Terminal 7 — rate와 불필요한 잔존 노드 확인

각 `rostopic hz`는 수 초 확인한 뒤 `Ctrl-C`로 종료한다.

```bash
source ~/.bashrc
ajou

rostopic hz /sensor/lidar1
rostopic hz /sensor/camera/front/compressed
rostopic hz /perception/object_recognition/detection/objects
rostopic hz /perception/object_recognition/tracking/objects
```

한 PC에서 MR과 BEVFusion을 함께 실행할 때의 건강한 기준은 대략 다음과 같다.

- LiDAR: `12--15 Hz` 권장. `8 Hz` 부근이면 simulator slowdown부터 조사한다.
- front camera: 설정값 `30 Hz`에 근접해야 한다.
- detection: LiDAR 입력률을 따른다.
- tracking: fixed-rate 약 `30 Hz`.

tracking-only 실험 전에 이전 풀스택 노드가 남지 않았는지 확인한다.

```bash
rosnode list | grep -E 'traffic_light|planning|purepursuit|rviz' || true
```

특히 `traffic_light_node`가 남으면 전방 camera를 추가 구독하고 CPU/GPU를 사용해 MORAI
실시간율을 낮출 수 있다. 정상적인 tracking-only 실행에서는 위 명령이 비어 있어야
한다.

### Terminal 8 — 경량 tracking rosbag

`RUN_TAG`는 객체와 반복 번호를 구분해 작성한다.

```bash
source ~/.bashrc
ajou

bash scripts/rosbag/record/rosbag_for_static_obstacle_tracking_eval.sh \
  bevfusion_static_vehicle_manual_01
```

`Recording to ...`가 출력된 뒤 MORAI에서 직접 주행한다. 종료는 recorder 터미널에서
`Ctrl-C` 한 번이다. 저장 경로는 다음과 같다.

```text
rosbags/eval/perception/static_obstacle_tracking/
```

경량 recorder는 raw camera/LiDAR를 bag에 쓰지 않지만 다음 사후 분석 정보는 보존한다.

- `/Ego_topic`, `/Competition_topic`, `/Object_topic`, collision.
- localization, TF.
- BEVFusion detection/status/latency.
- tracking measurement/output, prediction.
- `/rosout`, diagnostics.

raw sensor로 detector를 다시 추론해야 할 때만 아래 full recorder를 사용한다.

```bash
bash scripts/rosbag/record/rosbag_for_static_obstacle_tracking_eval_full.sh \
  bevfusion_static_vehicle_manual_full_01
```

## 종료 순서와 잔존 프로세스 확인

다음 순서로 각 명령을 실행한 원래 터미널에서 `Ctrl-C`한다.

```text
recorder -> tracking -> bevfusion -> localization -> sensing -> udp -> roscore
```

`traffic`은 이 profile에서 처음부터 실행하지 않는다. 이전 실행의 traffic launch가
남았다면 부모 roslaunch를 종료한다.

```bash
pkill -INT -f 'roslaunch traffic_light_detection traffic_light_node.launch'
```

5--10초 뒤에도 남을 때만 다음을 사용한다.

```bash
pkill -TERM -f 'traffic_light_detection'
rosnode cleanup
```

마지막 확인:

```bash
rosnode list
docker compose -f docker/bevfusion/compose.yaml ps
```

## 정보의 기준

이 runbook과 구현이 충돌하면 다음 순서로 확인한다.

1. alias 실제값: `docs/common_alias.txt`.
2. UDP 역할: `base_ws/src/sensor_interface/launch/morai_udp_interface.launch`.
3. sensor TF 역할: `base_ws/src/bringup/launch/sensing.launch`.
4. tracking 계약: `base_ws/src/perception_object_pipeline/launch/object_pipeline.launch`.
5. 녹화 토픽: `scripts/rosbag/record/rosbag_for_static_obstacle_tracking_eval.sh`.
