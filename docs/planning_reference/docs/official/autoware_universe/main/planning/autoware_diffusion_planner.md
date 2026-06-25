---
source_type: rendered_autoware_docs
site: autoware_universe
title: Autoware Diffusion Planner
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_diffusion_planner/
rendered_html: rendered/autoware_universe_html/index-154.html
generated: 2026-06-24T14:59:31.561186+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Autoware Diffusion Planner

## Overview

The **Autoware Diffusion Planner** is a trajectory generation module for autonomous vehicles, designed to work within the [Autoware](https://autoware.org/) ecosystem. It leverages the [Diffusion Planner](https://github.com/ZhengYinan-AIR/Diffusion-Planner) model, as described in the paper ["Diffusion-Based Planning for Autonomous Driving with Flexible Guidance"](https://arxiv.org/abs/2501.15564) by Zheng et al.

This planner generates smooth, feasible, and safe trajectories by considering:

- Dynamic and static obstacles
- Vehicle kinematics
- User-defined constraints
- Lanelet2 map context
- Traffic signals and speed limits

It is implemented as a ROS 2 component node, making it easy to integrate into Autoware-based stacks. The node is aimed at working within the proposed [Autoware new planning framework](https://github.com/tier4/new_planning_framework).

---

## How to use

### (1) Prerequisites

Make sure that the directory specified in `planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml` points to the correct model version and contains the required model weight and parameter files.

```
$ ls ~/autoware_data/ml_models/diffusion_planner/v4.0/
diffusion_planner.onnx diffusion_planner.param.json
```

This can be downloaded by following [Download artifacts](https://github.com/autowarefoundation/autoware/blob/main/ansible/roles/artifacts/README.md#download-artifacts).

### (2) Launch the planning simulator

Pass `planning_setting:=diffusion_planner` to switch the planning stack from the rule-based scenario planner to the diffusion planner. This argument automatically swaps the trajectory generator, the planning validator input topic, and the diagnostics graph, so no additional launch-file edits are required.

```
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/path/to/your/map \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  planning_setting:=diffusion_planner
```

## Features

- **Diffusion-based trajectory generation** for flexible and robust planning

  [![Diffusion-Based trajectory generation](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_diffusion_planner/media/diffusion_planner.gif)](media/diffusion_planner.gif)

- **Integration with Lanelet2 maps** for lane-level context

  [![Lanelet Map Integration](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_diffusion_planner/media/lanelet_map_integration.png)](media/lanelet_map_integration.png)

- **Dynamic and static obstacle handling** using perception inputs

  [![Static Agent Reaction](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_diffusion_planner/media/diffusion_planner_reacts_to_bus.gif)](media/diffusion_planner_reacts_to_bus.gif)

  [![Diffusion Planner](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_diffusion_planner/media/reaction_to_other_agents.gif)](media/reaction_to_other_agents.gif)

- **Traffic signal and speed limit awareness**

  [![Traffic Light Support](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_diffusion_planner/media/traffic_light_support.gif)](media/traffic_light_support.gif)

- **ONNX Runtime** inference for fast neural network execution
- **ROS 2 publishers** for planned trajectories, predicted objects, and debug markers

---

## Parameters

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| plugins\_path | string | Path to libautoware\_tensorrt\_plugins.so file for native TensorRT and ONNX Runtime TensorRT EP | $(find-pkg-share autoware\_tensorrt\_plugins)/plugins/libautoware\_tensorrt\_plugins.so | N/A |
| model.type | string | Inference model type | single\_step | ['single\_step', 'multi\_step'] |
| model.args\_path | string | Path to model argument/configuration file | $(var data\_path)/diffusion\_planner/v5.0/args.json | N/A |
| model.backend | string | Inference backend. tensorrt uses the native TensorRT implementation; ort\_cpu/ort\_cuda/ort\_tensorrt use ONNX Runtime with the corresponding execution provider. | tensorrt | ['tensorrt', 'ort\_cpu', 'ort\_cuda', 'ort\_tensorrt'] |
| model.precision | string | TensorRT engine precision | fp32 | ['fp32', 'fp16'] |
| model.use\_cuda\_graph | boolean | Use CUDA Graph replay for TensorRT enqueue calls | True | N/A |
| single\_step\_model.onnx\_model\_path | string | Path to the single-step ONNX model file | $(var data\_path)/diffusion\_planner/v5.0/diffusion\_planner.onnx | N/A |
| multi\_step\_model.encoder\_onnx\_model\_path | string | Path to the encoder ONNX model file | $(var data\_path)/diffusion\_planner/v5.0/diffusion\_planner\_encoder.onnx | N/A |
| multi\_step\_model.decoder\_onnx\_model\_path | string | Path to the decoder ONNX model file | $(var data\_path)/diffusion\_planner/v5.0/diffusion\_planner\_decoder.onnx | N/A |
| multi\_step\_model.turn\_indicator\_onnx\_model\_path | string | Path to the turn indicator ONNX model file | $(var data\_path)/diffusion\_planner/v5.0/diffusion\_planner\_turn\_indicator.onnx | N/A |
| multi\_step\_model.dpm\_solver\_steps | integer | Number of DPM-Solver denoising steps. The final denoise-to-zero step is added separately. | 10 | ≥2 |
| planning\_frequency\_hz | float | Planning frequency in Hz | 10.0 | >0 |
| ignore\_neighbors | boolean | Ignore neighbor agents | False | N/A |
| ignore\_unknown\_neighbors | boolean | Ignore neighbor agents with unknown class | True | N/A |
| traffic\_light\_group\_msg\_timeout\_seconds | float | Timeout for traffic light group messages (seconds) | 0.2 | >0 |
| build\_only | boolean | On build only, do not run the planner | False | N/A |
| batch\_size | integer | Batch size for multi-batch inference | 1 | ≥1 |
| temperature | array | List of temperature for sampling trajectories. Its length must be the same as `batch_size`. Each value means the coefficient of the standard normal distribution which is used for each initial noise of corresponding batch index. val = N(0,1) \* temperature | [0.0] | N/A |
| velocity\_smoothing\_window | integer | Window size for velocity smoothing. Set to 1 to disable smoothing. | 8 | ≥1 |
| stopping\_threshold | float | Threshold for keeping the stopping state [m/s] | 0.3 | N/A |
| turn\_indicator\_keep\_offset | float | Offset to add to the KEEP turn indicator logit for bias adjustment | -1.25 | N/A |
| turn\_indicator\_hold\_duration | float | Duration to hold the turn indicator command before allowing changes (seconds) | 1.0 | N/A |
| shift\_x | boolean | Whether to shift the ego center position by half the wheel base | False | N/A |
| delay\_step | integer | Input delay step for inference | 0 | ≥0 |
| line\_string\_max\_step\_m | float | Maximum arc-length step per model point when resampling line strings [m]. Smaller values produce more fixed-size sub-segments per line string. | 5.0 | >0 |
| use\_time\_interpolation | boolean | Whether to interpolate ego past trajectory at regular time intervals | False | N/A |
| start\_guidance.reference\_distance\_m | float | Reference straight-line distance between the ego trajectory start and terminal point [m] | 10.0 | ≥0 |
| start\_guidance.max\_scale | float | Maximum scale applied to the ego trajectory by start guidance | 30.0 | ≥1.0 |
| stop\_guidance.stop\_acceleration\_mps2 | float | Deceleration magnitude used to calculate stop guidance reference distance [m/s^2] | 1.0 | >0 |
| centerline\_guidance.start\_time\_s | float | Trajectory points at and after this prediction time are snapped to the nearest route centerline point [s] | 2.0 | ≥0 |
| planning\_factor.enable\_stop | boolean | Enable stop planning factor | False | N/A |
| planning\_factor.enable\_slowdown | boolean | Enable slowdown planning factor | False | N/A |
| planning\_factor.stop\_velocity\_threshold | float | Velocity threshold to detect a stop [m/s] | 0.1 | N/A |
| planning\_factor.stop\_keep\_duration\_threshold | float | Duration threshold to validate a stop [s] | 1.0 | N/A |
| planning\_factor.slowdown\_accel\_threshold | float | Acceleration threshold to detect a slowdown [m/s^2] | -0.3 | N/A |
| debug\_params.publish\_debug\_map | boolean | Publish debug map markers | False | N/A |
| debug\_params.publish\_debug\_route | boolean | Publish debug route markers | True | N/A |
| debug\_params.publish\_debug\_linestrings | boolean | Publish debug linestring markers | True | N/A |

Parameters can be set via YAML (see `config/diffusion_planner.param.yaml`).

---

## Inputs

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/input/odometry` | nav\_msgs/msg/Odometry | Ego vehicle odometry |
| `~/input/acceleration` | geometry\_msgs/msg/AccelWithCovarianceStamped | Ego acceleration |
| `~/input/tracked_objects` | autoware\_perception\_msgs/msg/TrackedObjects | Detected dynamic objects |
| `~/input/traffic_signals` | autoware\_perception\_msgs/msg/TrafficLightGroupArray | Traffic light states |
| `~/input/vector_map` | autoware\_map\_msgs/msg/LaneletMapBin | Lanelet2 map |
| `~/input/route` | autoware\_planning\_msgs/msg/LaneletRoute | Route information |
| `~/input/turn_indicators` | autoware\_vehicle\_msgs/msg/TurnIndicatorsReport | Turn indicator information |

## Outputs

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/output/trajectory` | autoware\_planning\_msgs/msg/Trajectory | Planned trajectory for the ego vehicle |
| `~/output/trajectories` | autoware\_internal\_planning\_msgs/msg/CandidateTrajectories | Multiple candidate trajectories |
| `~/output/predicted_objects` | autoware\_perception\_msgs/msg/PredictedObjects | Predicted future states of dynamic objects |
| `~/output/turn_indicators` | autoware\_vehicle\_msgs/msg/TurnIndicatorsCommand | Planned turn indicator command |
| `~/output/debug/traffic_signal` | autoware\_perception\_msgs/msg/TrafficLightGroup | First traffic light on route (ego forward) for RViz/ad\_api |
| `~/debug/lane_marker` | visualization\_msgs/msg/MarkerArray | Lane debug markers |
| `~/debug/route_marker` | visualization\_msgs/msg/MarkerArray | Route debug markers |

---

## Testing

Unit tests are provided and can be run with:

```
colcon test --packages-select autoware_diffusion_planner
colcon test-result --all
```

---

## ONNX Model and Versioning

The Diffusion Planner relies on an ONNX model for inference.
To ensure compatibility between models and the ROS 2 node implementation, the model versioning scheme follows **major** and **minor** numbers:
The model version is defined either by the directory name provided to the node or within the `diffusion_planner.param.json` configuration file.

- **Major version**
  Incremented when there are changes in the model **inputs/outputs or architecture**.

  > ![⚠](https://cdn.jsdelivr.net/gh/jdecked/twemoji@14.1.2/assets/svg/26a0.svg ":warning:") Models with different major versions are **not compatible** with the current ROS node.

- **Minor version**
  Incremented when **only the weight files are updated**.
  As long as the major version matches, the node remains compatible, and the new model can be used directly.

To download the latest model, follow [Download artifacts](https://github.com/autowarefoundation/autoware/blob/main/ansible/roles/artifacts/README.md#download-artifacts).

### Model Version History

| Version | Release Date | Notes | ROS Node Compatibility |
| --- | --- | --- | --- |
| **0.1** | 2025/07/05 | - First public release - Route planning based on TIER IV real data | NG |
| **1.0** | 2025/09/12 | - Route Termination learning - Output turn-signal (indicator)  - Lane type integration in HD map for improved accuracy - Added datasets:   - Synthetic Data: **4.0M points**   - Real Data: **1.5M points** | NG |
| **2.0** | 2025/11/26 | - Increased the number of acceptable lane types ("crosswalk", "pedestrian\_lane" and "walkway") for left and right boundaries.  - Added `Polygon` and `LineString` as acceptable input types.  - Increased the maximum length of each history record to 3 seconds.  - Added support for turn\_indicator as an input (this is just an interface, not used in v2.0 weights).  - Increased `NUM_SEGMENTS_IN_LANE` from 70 to 140. | NG |
| **3.0** | 2026/01/09 | - Added `TURN_INDICATOR_OUTPUT_KEEP` to allow the model to focus on the timing of status change.  - Conducted Supervised Fine-Tuning (SFT) with carefully filtered data.  - Increased the encoder layers from 3 to 6. | OK |
| **3.1** | 2026/03/05 | - ONNX simplified model for faster TRT engine build and reduced GPU memory.  - Same weights as v3.0 (no retraining). | OK |
| **4.0** | 2026/03/23 | - Added `delay` input for Real-Time Chunking (RTC): reuses first N timesteps from the previous prediction for trajectory continuity.  - Added one-hot type encoding for polygons (`intersection_area`) and line strings (`stop_line`, `road_border`).  - Increased `NUM_LINE_STRINGS` from 10 to 60.  - Added line string resampling (`line_string_max_step_m`).  - Added debug visualization for line strings. | OK |

---

## Development & Contribution

- Follow the [Autoware coding guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/).
- Contributions, bug reports, and feature requests are welcome via GitHub issues and pull requests.

---

## References

- [Diffusion Planner (original repo)](https://github.com/ZhengYinan-AIR/Diffusion-Planner)
- [Diffusion planner (our fork of the previous repo, used to train the model)](https://github.com/tier4/Diffusion-Planner)
- ["Diffusion-Based Planning for Autonomous Driving with Flexible Guidance"](https://arxiv.org/abs/2309.00615)

---

## License

This package is released under the Apache 2.0 License.
