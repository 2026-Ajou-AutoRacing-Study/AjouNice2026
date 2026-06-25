---
source_type: rendered_autoware_docs
site: autoware_core
title: autoware_planning_factor_interface
canonical_url: https://autowarefoundation.github.io/autoware_core/main/planning/autoware_planning_factor_interface/
rendered_html: rendered/autoware_core_html/index-60.html
generated: 2026-06-24T15:00:04.878586+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# autoware\_planning\_factor\_interface

## Overview

The `PlanningFactorInterface` is a C++ class designed to facilitate the addition and publication of planning factors.

## Design

The `PlanningFactorInterface` class is designed to be lightweight and efficient, with the following key components:

- **Add:** Methods to add planning factors to the interface.

- **Publisher:** The class includes a publisher for `PlanningFactorArray` messages, which are used to distribute planning factors to other nodes in the system.

The design emphasizes flexibility and ease of use, allowing developers to quickly integrate new planning factors into autoware.

## Usage

### Including the Header

To use the `PlanningFactorInterface`, include the header file in your code:

```cpp

# include

```

### Creating an Instance

Instantiate the `PlanningFactorInterface` by providing a node and a name for the factor module:

```cpp

class AvoidancePlanner
{
public:
AvoidancePlanner(rclcpp::Node & node)
: planning\_factor\_interface\_{std::make\_unique<
autoware::planning\_factor\_interface::PlanningFactorInterface>(
&node, "avoidance\_planner")}
```

You can also enable console output for debugging by setting the appropriate parameters:

```
// Enable console output with a 1000ms throttle duration
planning_factor_interface_ = std::make_unique<
  autoware::planning_factor_interface::PlanningFactorInterface>(
  &node, "avoidance_planner", true, 1000);
```

### Adding Planning Factors

```
planning_factor_interface_->add(
        traj_points, ego_pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::NONE,
        autoware_internal_planning_msgs::msg::SafetyFactorArray{});
```

### Publishing Factors

After adding planning factors, you can publish them by calling the `publish` method:

```
// Publish the added factors
planning_factor_interface_->publish();
```
