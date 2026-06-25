---
source_type: rendered_autoware_docs
site: autoware_documentation
title: Traffic light recognition simulation
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/traffic-light/
rendered_html: rendered/autoware_documentation_html/index-63.html
generated: 2026-06-24T15:00:01.490693+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Traffic light recognition simulation

By default, traffic lights on the map are all treated as if they are set to green. As a result, when a path is created that passed through an intersection with a traffic light, the ego vehicle will drive through the intersection without stopping.

The following steps explain how to set and reset traffic lights in order to test how the Planning component will respond.

## Set traffic light

By default, Rviz doesn't display the IDs of traffic lights on the map.
To enable the display of traffic light IDs, follow these steps:

1. In the `Displays` panel, find the `traffic_light_id` topic by toggling the triangle icons next to `Map > Lanelet2VectorMap > Namespaces`.
2. Check the `traffic_light_id` checkbox.
3. Reload the topic by clicking the `Map` checkbox twice.
4. Have a closer look at the IDs by zooming in the region or by changing the View type.

![see-traffic-light-ID](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/traffic-light/see-traffic-light-ID.png)

1. Go to `Panels -> Add new panel`, select `tier4_traffic_light_rviz_plugin/TrafficLightPublishPanel`, and then press `OK`.
2. In `TrafficLightPublishPanel`, set the `ID` and color of the traffic light.
3. Click the `SET` button.
   ![set-traffic-light](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/traffic-light/set-traffic-light.png)
4. Finally, click the `PUBLISH` button to send the traffic light status to the simulator. Any planned path that goes past the selected traffic light will then change accordingly.

![send-traffic-light-color](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/traffic-light/send-traffic-light-color.png)

## Update/Reset traffic light

You can update the color of the traffic light by selecting the next color (in the image it is `GREEN`) and clicking `SET` button. In the image the traffic light in front of the ego vehicle changed from `RED` to `GREEN` and the vehicle restarted.

![after-traffic-light-color-update](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/traffic-light/after-traffic-light-color-update.png)

To remove a traffic light from `TrafficLightPublishPanel`, click the `RESET` button.
