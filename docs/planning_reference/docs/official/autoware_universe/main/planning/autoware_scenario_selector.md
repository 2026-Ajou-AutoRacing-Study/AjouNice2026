---
source_type: rendered_autoware_docs
site: autoware_universe
title: autoware_scenario_selector
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_scenario_selector/
rendered_html: rendered/autoware_universe_html/index-169.html
generated: 2026-06-24T14:59:32.476357+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware\_scenario\_selector

## scenario\_selector\_node

`scenario_selector_node` is a node that switches trajectories from each scenario.

### Input topics

| Name | Type | Description |
| --- | --- | --- |
| `~input/lane_driving/trajectory` | autoware\_planning\_msgs::Trajectory | trajectory of LaneDriving scenario |
| `~input/parking/trajectory` | autoware\_planning\_msgs::Trajectory | trajectory of Parking scenario |
| `~input/lanelet_map` | autoware\_map\_msgs::msg::LaneletMapBin |  |
| `~input/route` | autoware\_planning\_msgs::LaneletRoute | route and goal pose |
| `~input/odometry` | nav\_msgs::Odometry | for checking whether vehicle is stopped |
| `is_parking_completed` | bool (implemented as rosparam) | whether all split trajectory of Parking are published |

### Output topics

| Name | Type | Description |
| --- | --- | --- |
| `~output/scenario` | tier4\_planning\_msgs::Scenario | current scenario and scenarios to be activated |
| `~output/trajectory` | autoware\_planning\_msgs::Trajectory | trajectory to be followed |

### Output TFs

None

### How to launch

1. Write your remapping info in `scenario_selector.launch` or add args when executing `roslaunch`
2. `roslaunch autoware_scenario_selector scenario_selector.launch`
   - If you would like to use only a single scenario, `roslaunch autoware_scenario_selector dummy_scenario_selector_{scenario_name}.launch`

### Parameters

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| update\_rate | float | timer's update rate | 10 | ≥0.0 |
| th\_max\_message\_delay\_sec | float | threshold time of input messages' maximum delay | 1 | ≥0.0 |
| th\_arrived\_distance\_m | float | threshold distance to check if vehicle has arrived at the trajectory's endpoint | 1 | ≥0.0 |
| th\_stopped\_time\_sec | float | threshold time to check if vehicle is stopped | 1 | ≥0.0 |
| th\_stopped\_velocity\_mps | float | threshold velocity to check if vehicle is stopped | 0.01 | ≥0.0 |
| enable\_mode\_switching | boolean | enable switching between scenario modes when ego is stuck in parking area | 1 | N/A |

### Flowchart

![uml diagram](../../../../assets/autoware_universe/main/planning/autoware_scenario_selector/image-01-4d1f910663.svg)

![uml diagram](../../../../assets/autoware_universe/main/planning/autoware_scenario_selector/image-02-67818d8bf5.svg)
