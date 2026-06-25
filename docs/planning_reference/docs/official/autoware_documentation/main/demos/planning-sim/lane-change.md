---
source_type: rendered_autoware_docs
site: autoware_documentation
title: Lane change scenario
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/lane-change/
rendered_html: rendered/autoware_documentation_html/index-58.html
generated: 2026-06-24T15:00:01.322990+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Lane change scenario

1. Download and unpack Nishishinjuku map.

   ```
   mkdir -p ~/autoware_data/maps
   gdown -O ~/autoware_data/maps/ 'https://github.com/autowarefoundation/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip'
   unzip -d ~/autoware_data/maps ~/autoware_data/maps/nishishinjuku_autoware_map.zip
   ```
2. Launch autoware with Nishishinjuku map with following command:

   ```
   source ~/autoware/install/setup.bash
   ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_data/maps/nishishinjuku_autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
   ```

   ![open-nishishinjuku-map](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/lane-change/open-nishishinjuku-map.png)
3. Set an initial pose and a goal pose in adjacent lanes.

   ![set-position-and-goal](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/lane-change/set-position-and-goal.png)
4. Engage the ego vehicle. It will make a lane change along the planned path.

   ![lane-changing](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/lane-change/lane-changing.png)
