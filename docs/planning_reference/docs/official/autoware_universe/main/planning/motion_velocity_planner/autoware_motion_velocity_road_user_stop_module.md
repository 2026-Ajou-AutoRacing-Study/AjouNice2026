---
source_type: rendered_autoware_docs
site: autoware_universe
title: Road User Stop
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module/
rendered_html: rendered/autoware_universe_html/index-224.html
generated: 2026-06-24T14:59:36.165912+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Road User Stop

## Role

The `road_user_stop` module stops the ego vehicle when pedestrians, cyclists, or other road users are detected on or near the road within the ego's planned trajectory. This module ensures safe interaction with vulnerable road users by maintaining appropriate stopping distances and making decisions about when to stop versus when to continue driving.

## Activation

This module is activated if the launch parameter `launch_road_user_stop_module` is set to true.

## Parameters

### Option Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| suppress\_sudden\_stop | [-] | bool | Enable to suppress sudden stop | true |

### Stop Planning Parameters

#### Longitudinal Margin Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| default\_margin | [m] | double | Default longitudinal margin to obstacle | 5.0 |
| terminal\_margin | [m] | double | Stop margin at the goal position | 3.0 |
| minimum\_margin | [m] | double | Minimum stop margin for behavior decision | 3.0 |

#### Opposing Traffic Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| stop\_margin | [m] | double | Ideal stop-margin from moving opposing obstacle when ego stops | 10.0 |
| max\_negative\_velocity | [m/s] | double | Maximum velocity of opposing traffic to consider stop planning | -0.1 |
| min\_velocity\_for\_stop\_planning | [m/s] | double | Minimum velocity of ego to consider stop planning | 2.77 |
| effective\_deceleration | [m/s^2] | double | Higher value brings final stop-margin closer to ideal value | 4.0 |

#### General Stop Planning Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| hold\_stop\_velocity\_threshold | [m/s] | double | The maximum ego velocity to hold stopping | 0.01 |
| hold\_stop\_distance\_threshold | [m] | double | The ego keeps stopping if distance to stop changes within threshold | 0.3 |
| limit\_min\_acc | [m/s^2] | double | Overwrite the deceleration limit (usually from common\_param.yaml) | -2.5 |
| sudden\_object\_acc\_threshold | [m/s^2] | double | If stop achievable by smaller deceleration, not "sudden stop" | -1.0 |
| sudden\_object\_dist\_threshold | [m] | double | If stop distance is longer than this, not considered "sudden stop" | 1000.0 |
| abandon\_to\_stop | [-] | bool | Give up stopping when cannot avoid run over within decel limit | false |

#### Stop on Curve Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| enable\_approaching | [-] | bool | Enable approaching behavior on curved paths | false |
| additional\_stop\_margin | [m] | double | Additional stop margin for obstacles on curves | 3.0 |
| min\_stop\_margin | [m] | double | Minimum stop margin on curves | 6.0 |

### Obstacle Filtering Parameters

#### Object Type Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| pedestrian | [-] | bool | Enable detection of pedestrians | true |
| bicycle | [-] | bool | Enable detection of bicycles | true |
| motorcycle | [-] | bool | Enable detection of motorcycles | false |
| unknown | [-] | bool | Enable detection of unknown objects | false |

#### General Filtering Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| trajectory\_lateral\_margin | [m] | double | Lateral margin from ego trajectory to detect objects | 1.0 |

#### Intersection Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| exclude | [-] | bool | If true, exclude objects inside intersection lanelets from detection | false |

#### Crosswalk Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| exclude | [-] | bool | Exclude objects near crosswalks | true |
| margin | [m] | double | Margin distance from crosswalk | 1.0 |

#### Opposing Traffic Detection Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| enable | [-] | bool | Enable wrong-way object detection | true |
| angle\_threshold | [deg] | double | Angle threshold for wrong-way detection | 150.0 |
| min\_speed\_threshold | [m/s] | double | Minimum speed for wrong-way detection | 0.5 |

#### Temporal Filtering Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| min\_detection\_duration | [s] | double | Minimum duration for object detection | 0.1 |
| lost\_object\_retention\_duration | [s] | double | Duration to keep tracking objects after they are lost or exit the detection area | 2.0 |
| polygon\_expansion\_length | [m] | double | Distance to expand object polygon outward from centroid when object was previously inside detection area | 0.5 |

### Stopped Object Tracking Parameters

| Name | Unit | Type | Description | Default value |
| --- | --- | --- | --- | --- |
| stopped\_velocity\_threshold | [m/s] | double | Velocity threshold to consider object as stopped | 0.5 |
| stopped\_position\_tolerance | [m] | double | Distance threshold for stopped position movement | 0.3 |
| ego\_reached\_wall\_distance\_threshold | [m] | double | Distance threshold to consider ego reached virtual wall | 0.5 |
| stopped\_duration\_after\_ego\_arrival | [s] | double | Duration threshold for stopped object after ego reaches virtual wall | 3.0 |

## Limitations

- If object tracking is lost for an extended period or object classification is misidentified, the module may not function properly and may fail to stop as expected
