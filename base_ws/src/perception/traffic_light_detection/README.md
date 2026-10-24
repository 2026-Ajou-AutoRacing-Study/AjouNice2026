# Traffic light detection runtime

현재 launch는 `launch/traffic_light_node.launch`다. 신호등 작업은 이 README와 실제 호스트의 resolved 인자에서 시작한다.
BEVFusion 학습·dataset 문서를 먼저 읽을 필요는 없다.

## 프로세스·Python 경계

| 프로세스 | 현재 계약 |
|---|---|
| `traffic_light_route_group_id` | ROS Noetic `/usr/bin/python3`·system Lanelet2로 active-route regulatory-element ID 선택 |
| `traffic_light_node` | `inference_python` interpreter의 YOLO GPU 추론 |
| Output | AW149 표준 `/perception/traffic_light_recognition/traffic_signals` |

팀 공유 기본 `inference_python`은 `/usr/bin/python3`다. Python 경로만으로 CPU/GPU를 단정하지 않는다.
이 RTX5060Ti 호스트는 Python3.11 CUDA runtime과 Python3.8 Lanelet2 binding을 분리한다.
planning 전체를 Conda에서 실행하지 말고 정상 sourced ROS shell을 사용한다.

```bash
roslaunch traffic_light_detection traffic_light_node.launch \
  inference_python:=$HOME/miniconda3/envs/traffic_light_gpu/bin/python device:=0
```

## 환경·입력

- GPU에 맞는 PyTorch wheel을 먼저 설치한 후 `requirements-runtime.txt`를 설치한다. CUDA/PyTorch는 rosdep 외부 계약이다.
- 이 호스트의 확인 이력은 Python3.11.15·PyTorch2.7.1+cu128·Torchvision0.22.1+cu128·NumPy1.26.4·OpenCV4.8.1이다. 다른 GPU에 동일 wheel을 강제하지 않는다.
- 팀 Ultralytics **8.4.143**을 유지한다. interpreter는 `scripts/workspace/source_aw149.sh` source 후 `rospy`·generated Autoware messages를 import해야 한다.
- 활성화 전에 `weights/best.pt` CUDA load를 확인한다. MORAI front 1280×720·검증된 10Hz 판단 cadence·confidence·temporal semantics를 유지한다.
- 이 호스트의 warm-up 추론 35–45ms는 과거 실측이며 다른 GPU·LAN 전체 지연 보증이 아니다.
- map 기본은 v87다. 부모 launch의 map/projector override도 확인한다. planning UDP time은 `use_sim_time=false`·기존 `is_simulation=false` 계약을 따른다.

## Competition signal policy

`config/traffic_signal_profiles.yaml`은 route matcher가 선택한 regulatory-element ID를 재사용 가능한 신호 program에 연결한다.
Profile은 supported lamps·유효 Autoware element 조합·detector correction·route-compatible fail-open phase를 정의한다.
출력은 표준 `TrafficLightGroup.elements[]`를 유지하며 A/B/C/D 동작을 detector code 변경 없이 설정한다.

Route helper의 invalid match는 group ID `0`을 발행하여 기존 distance fallback으로 즉시 전환한다.
별도 2초 message timeout은 helper process 상실에만 적용한다. 옛 route ID를 계속 유지하지 않는다.
Planning `competition_fail_open`도 missing/stale/entirely unknown 신호에서 영구 정지를 피하도록 허용한다.
두 fail-open 정책은 **simulation competition 전용**이며 실제 차량 안전 정책이 아니다.
원시 correction 이유는 `/traffic_light_node/debug_policy_reason`에서 확인한다.

환경 설치 명령과 정리 전 상세는 [보존 원문](docs/legacy/context_261024/README.md)에 있다.
