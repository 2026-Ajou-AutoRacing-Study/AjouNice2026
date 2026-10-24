# Traffic light detection runtime

[인지 작업 안내](../../../../docs/perception/README.md)에서 신호등 작업을 선택한다.
3D detector 학습·dataset 문서를 먼저 읽을 필요는 없다. 아래 설치 이력과 실제 대상 호스트의
Python·CUDA·device 설정을 구분하고 현재 launch를 확인한다.

This package publishes
`/perception/traffic_light_recognition/traffic_signals` for the AW149 planning
stack. Its Python runtime is intentionally kept outside rosdep because the
correct PyTorch wheel depends on the target GPU architecture.

## Runtime split

The launch deliberately runs two processes:

- `traffic_light_route_group_id` uses ROS Noetic's `/usr/bin/python3` and the
  system Lanelet2 binding to select the next traffic-light regulatory-element
  ID on the active route.
- `traffic_light_node` uses the Python executable passed through
  `inference_python` and performs YOLO inference on the GPU.

This split is required on the RTX 5060 Ti host. Its CUDA-capable PyTorch uses
Python 3.11, while the Ubuntu 20.04 Lanelet2 Python extension is built for
Python 3.8. Running the complete planning stack in either Conda environment is
unsupported; launch planning from the normal sourced ROS shell.

The shared planning default remains the system interpreter used by the
planning team:

```text
/usr/bin/python3
```

Hosts that require a newer CUDA/PyTorch runtime can override it without editing
the launch file:

```bash
roslaunch traffic_light_detection traffic_light_node.launch \
  inference_python:=$HOME/miniconda3/envs/traffic_light_gpu/bin/python device:=0
```

## Environment

On this RTX 5060 Ti host the isolated environment was created from the already
validated CUDA 12.8 PointPainting environment, then reduced to an independent
runtime contract:

```bash
conda create -n traffic_light_gpu --clone openpcdet_rtx50
conda activate traffic_light_gpu
python -m pip install -r \
  base_ws/src/perception/traffic_light_detection/requirements-runtime.txt
```

The RTX 5060 Ti runtime uses Python 3.11.15, PyTorch 2.7.1+cu128,
Torchvision 0.22.1+cu128, NumPy 1.26.4, and OpenCV 4.8.1. Keep the traffic
team's Ultralytics 8.4.143 version across hosts. Another GPU may use a different
PyTorch/CUDA build, but the `inference_python` interpreter must import `rospy`
and the generated Autoware messages after `scripts/workspace/source_aw149.sh`
is sourced.

Install a platform-appropriate PyTorch build first, then install the remaining
locked runtime:

```bash
python3 -m pip install -r requirements-runtime.txt
```

The GPU integration baseline processes the live 1280x720 MORAI front camera at
the traffic-light team's validated 10 Hz decision cadence; warmed-up inference
takes about 35--45 ms per frame on the RTX 5060 Ti. GPU enablement must not
change the validated confidence or temporal decision semantics. The route
helper publishes group ID `0` when its match is invalid, which makes the GPU
node immediately use the existing distance fallback instead of retaining an
old route ID. The separate 2-second message timeout only covers loss of the
helper process itself.

새 환경에서는 `start_traffic_light` 활성화 전에 `weights/best.pt`의 CUDA 로드를 확인한다.
현재 launch의 지도 기본값은 v87이며 부모 launch가 map/projector를 override할 수 있다.
실제 resolved 인자를 확인한다. `morai_planning_stack.launch`의 UDP 시간 계약은
`use_sim_time:=false`이며 planning의 기존 `is_simulation:=false` 동작을 유지한다.

## Competition signal phase profiles

`config/traffic_signal_profiles.yaml` maps the Lanelet traffic-light regulatory
element ID selected by the route matcher to a reusable signal-program profile.
Each profile declares supported lamps, valid Autoware element combinations,
detector corrections, and a route-compatible fail-open phase. This keeps the
published message as standard `TrafficLightGroup.elements[]` while allowing
competition-specific A/B/C/D behavior to be changed without editing detector
code. The policy is intentionally fail-open for simulation competition use and
must not be reused as a real-vehicle safety policy.

The planning traffic-light module also enables `competition_fail_open`. Missing,
stale, or entirely unknown signal input therefore permits motion instead of
creating a permanent stop. Both controls are competition-only availability
choices; the raw correction reason remains observable on
`/traffic_light_node/debug_policy_reason`.
