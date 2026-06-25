---
source_type: rendered_autoware_docs
site: autoware_documentation
title: Lane driving scenario
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/lane-driving/
rendered_html: rendered/autoware_documentation_html/index-59.html
generated: 2026-06-24T15:00:01.349822+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Lane driving scenario

## 1. Launch Autoware

```
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_data/maps/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

Warning

Note that you cannot use `~` instead of `$HOME` here.

If `~` is used, the map will fail to load.

![after-autoware-launch](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/lane-following/after-autoware-launch.png)

## 2. Set an initial pose for the ego vehicle

![set-initial-pose](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/lane-following/set-initial-pose.png)

a) Click the `2D Pose estimate` button in the toolbar, or hit the `P` key.

b) In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the initial pose. An image representing the vehicle should now be displayed.

Warning

Remember to set the initial pose of the car in the same direction as the lane.

To confirm the direction of the lane, check the arrowheads displayed on the map.

## 3. Set a goal pose for the ego vehicle

a) Click the `2D Goal Pose` button in the toolbar, or hit the `G` key.

b) In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the goal pose. If done correctly, you will see a planned path from initial pose to goal pose.

![set-goal-pose](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/lane-following/set-goal-pose.png)

## 4. Start the ego vehicle

Now you can start the ego vehicle driving by clicking the `Auto` button in the `AutowareStatePanel`.
Alternatively, you can manually start the vehicle by running the following command:

```
source ~/autoware/install/setup.bash
ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode {}
```

After that, `Auto` button will be selected and grayed out.

![start-driving](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/lane-following/start-driving.png)
