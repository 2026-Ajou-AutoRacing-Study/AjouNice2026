---
source_type: rendered_autoware_docs
site: autoware_documentation
title: Placing dummy objects
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/placing-objects/
rendered_html: rendered/autoware_documentation_html/index-61.html
generated: 2026-06-24T15:00:01.438321+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Placing dummy objects

1. Click the `2D Dummy Car` or `2D Dummy Pedestrian` button in the toolbar.
2. Set the pose of the dummy object by clicking and dragging on the map.
3. Set the velocity of the object in `Tool Properties -> 2D Dummy Car/Pedestrian` panel.

Note

Changes to the `velocity` parameter will only affect objects placed after the parameter is changed.

![set-dummy-car](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/lane-following/place-dummy-car.png)

Delete all dummy objects

Click the `Delete All Objects` button in the toolbar and then click anywhere in the 3D View pane to complete it.

## Placing interactive dummy objects

Click the `Interactive` button in the toolbar to create the next dummy object in interactive mode.

![set-interactive-dummy-car](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/lane-following/check-interactive.png)

Info

- **Add an interactive dummy object:** `Shift`+`🖱️ Right Click` on the map.
- **Delete an interactive dummy object:** `Alt`+`🖱️ Right Click` on the object.
- **Move an existing interactive dummy object:** `🖱️ Right Click` and hold on the object. Then drag and drop it to the desired location.
