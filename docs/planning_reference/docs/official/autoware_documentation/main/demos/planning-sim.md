---
source_type: rendered_autoware_docs
site: autoware_documentation
title: Planning simulation
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/
rendered_html: rendered/autoware_documentation_html/index-55.html
generated: 2026-06-24T15:00:01.238468+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Planning simulation

## Preparation

### Download the sample map

Use the [`demo_artifacts`](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/demo_artifacts) ansible role to download and extract the sample map into `~/autoware_data/maps/sample-map-planning/`:

```
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
ansible-playbook autoware.dev_env.install_dev_env --tags demo_artifacts --ask-become-pass
```

Info

Sample map: Copyright 2020 TIER IV, Inc.

### Make sure the ML model artifacts are downloaded

Check if you have `~/autoware_data/ml_models` folder and files in it.

```
$ cd ~/autoware_data/ml_models
$ ls -C -w 30
bevfusion
calibration_status_classifier
camera_streampetr
diffusion_planner
image_projection_based_fusion
lidar_apollo_instance_segmentation
lidar_centerpoint
lidar_frnet
lidar_transfusion
ptv3
simpl_prediction
tensorrt_bevdet
tensorrt_rtmdet
tensorrt_yolox
traffic_light_classifier
traffic_light_fine_detector
vad
yabloc_pose_initializer
```

If not, please, follow [Manual downloading of artifacts](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts).

## Launch Autoware Planning Simulator

```
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_data/maps/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

Warning

Note that you cannot use `~` instead of `$HOME` here.

If `~` is used, the map will fail to load.

---

[Using Autoware Launch GUI](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/using-launch-gui/)

If you prefer a graphical user interface (GUI) over the command line for launching and managing your simulations, refer to the [Using Autoware Launch GUI](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/using-launch-gui/) section at the end of this document for a step-by-step guide.

---

[Reference Video Tutorials](https://drive.google.com/file/d/1bs_dX1JJ76qHk-SGvS6YF9gmekkN8fz7/view?usp=sharing)

---

## Basic simulations

[Lane Driving](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/lane-driving/)
[Parking](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/parking/)
[Pull out and pull over](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/pull-over-out/)
[Lane Change](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/lane-change/)

## Advanced simulations

[Placing Dummy Objects](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/placing-objects/)
[Avoidance](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/avoidance/)
[Traffic Light Recognition Simulation](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/traffic-light/)
[Driving Through a Crosswalk](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/crosswalk/)

## Increase the maximum velocity

The original Autoware is designed to operate at a wide speed range. But for safety reasons, the default maximum velocity has been limited to **15 km/h**.
Because of this, even if you drag the slider to a higher speed in the RViz panel, the system will not allow it.

To run Autoware at a higher speed, you can modify the `max_vel` parameter in the config file [autoware\_launch/config/planning/scenario\_planning/common/common.param.yaml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/common/common.param.yaml) located in the `autoware_launch` repository.

Example

Set `max_vel` to `20.0` (20 m/s = 72 km/h).
Then launch the planning simulator, place the vehicle and set the velocity limit with the slider.

![increase-max-velocity](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/others/increase-max-velocity.png)

## Create your own map

The content above takes place in the planning simulator using a sample map. If you are interested in running Autoware with maps of your own environment, please visit the [How to Create Vector Map](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/integrating-autoware/creating-maps/) section for guidance.
