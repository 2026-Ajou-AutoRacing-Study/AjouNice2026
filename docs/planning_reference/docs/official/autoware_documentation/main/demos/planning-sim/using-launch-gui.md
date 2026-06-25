---
source_type: rendered_autoware_docs
site: autoware_documentation
title: Planning simulator demo with Autoware Launch GUI
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/using-launch-gui/
rendered_html: rendered/autoware_documentation_html/index-64.html
generated: 2026-06-24T15:00:01.629394+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Planning simulator demo with Autoware Launch GUI

This section provides a step-by-step guide on using the Autoware Launch GUI for planning simulations, offering an alternative to the command-line instructions provided in the Basic simulations section.

## Getting Started with Autoware Launch GUI

1. **Installation:** Ensure you have installed the Autoware Launch GUI. [Installation instructions](https://github.com/autowarefoundation/autoware-launch-gui#installation).
2. **Launching the GUI:** Open the Autoware Launch GUI from your applications menu.

   ![GUI screenshot for launching the GUI](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/launch-gui/launch_gui_main.png)

## Launching a Planning Simulation

### Lane Driving Scenario

1. **Set Autoware Path:** In the GUI, set the path to your Autoware installation.

   ![GUI_screenshot_for_setting_Autoware_path](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/launch-gui/launch_gui_setup.png)
2. **Select Launch File:** Choose `planning_simulator.launch.xml` for the lane driving scenario.

   ![GUI screenshot for selecting launch file](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/launch-gui/selecting_launch_file.png)
3. **Customize Parameters:** Adjust parameters such as `map_path`, `vehicle_model`, and `sensor_model` as needed.

   ![GUI screenshot for customizing parameters](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/launch-gui/customizing-parameters1.png)
   ![GUI screenshot for customizing parameters](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/launch-gui/customizing-parameters2.png)
4. **Start Simulation:** Click the launch button to start the simulation.

   ![GUI screenshot for starting simulation](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/images/launch-gui/starting_simulation.png)
5. **Any Scenario:** From here, you can follow the instructions in the [Planning Scenario Simulations](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/#basic-simulations).

## Monitoring and Managing the Simulation

- **Real-Time Monitoring:** Use the GUI to monitor CPU/Memory usage and Autoware logs in real-time.
- **Profile Management:** Save your simulation profiles for quick access in future simulations.
- **Adjusting Parameters:** Easily modify simulation parameters on-the-fly through the GUI.
