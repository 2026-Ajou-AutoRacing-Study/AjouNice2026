---
source_type: rendered_autoware_docs
site: autoware_documentation
title: Parking scenario
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/parking/
rendered_html: rendered/autoware_documentation_html/index-60.html
generated: 2026-06-24T15:00:01.408907+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Parking scenario

1. Set an initial pose and a goal pose, and engage the ego vehicle.

   ![after-set-goal-pose](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/parking/after-set-goal-pose.png)
2. When the vehicle approaches the goal, it will switch from lane driving mode to parking mode.
3. After that, the vehicle will maneuver into the destination parking spot.

   ![parking-maneuver](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/parking/parking-maneuver.png)
