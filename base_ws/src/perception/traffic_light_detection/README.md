# Traffic light detection runtime

This package publishes
`/perception/traffic_light_recognition/traffic_signals` for the AW149 planning
stack. Its Python runtime is intentionally kept outside rosdep because the
correct PyTorch wheel depends on the target CPU/CUDA environment.

Install a platform-appropriate PyTorch 2.4.1 build, then install the remaining
locked runtime:

```bash
python3 -m pip install -r requirements-runtime.txt
```

The integration baseline was exercised with Python 3.8, Ultralytics 8.4.143,
PyTorch 2.4.1+cu121, and Torchvision 0.19.1. A clean install must load
`weights/best.pt` before enabling `start_traffic_light` in the integration
launch. The v61 Lanelet2 map and projector are supplied by the parent launch.
