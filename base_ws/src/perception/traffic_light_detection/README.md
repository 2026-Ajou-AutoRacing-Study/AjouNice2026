# Traffic light detection runtime

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

A clean install must load `weights/best.pt` on CUDA before enabling
`start_traffic_light`. The v62 Lanelet2 map and projector are supplied by the
parent launch. `morai_planning_stack.launch` keeps `use_sim_time:=false` for the
UDP clock and therefore preserves the planning team's existing
`is_simulation:=false` behavior.
