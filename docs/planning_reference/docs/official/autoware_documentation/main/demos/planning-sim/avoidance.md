---
source_type: rendered_autoware_docs
site: autoware_documentation
title: Avoidance scenario
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/avoidance/
rendered_html: rendered/autoware_documentation_html/index-56.html
generated: 2026-06-24T15:00:01.263154+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Avoidance scenario

1. Set an initial pose and a goal pose in the same lane. A path will be planned.

   ![set-position-and-goal](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/avoidance/set-position-and-goal.png)
2. Set a "2D Dummy Bus" on the roadside. A new path will be planned.

   ![set-dummy-bus](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/avoidance/set-dummy-bus.png)
3. Engage the ego vehicle. It will avoid the obstacle along the newly planned path.

   ![avoidance-driving](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/avoidance/avoidance-driving.png)
