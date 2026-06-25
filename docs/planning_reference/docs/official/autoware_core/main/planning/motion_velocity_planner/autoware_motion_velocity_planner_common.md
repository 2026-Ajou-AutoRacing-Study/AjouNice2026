---
source_type: rendered_autoware_docs
site: autoware_core
title: Motion Velocity Planner Common
canonical_url: https://autowarefoundation.github.io/autoware_core/main/planning/motion_velocity_planner/autoware_motion_velocity_planner_common/
rendered_html: rendered/autoware_core_html/index-69.html
generated: 2026-06-24T15:00:05.133330+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Motion Velocity Planner Common

This package provides common utilities and data structures for the motion velocity planner in the Autoware system. It contains tools for geometric calculations, trajectory processing, and velocity planning results.

## Overview

Package motion velocity planner is responsible for generating velocity profiles for autonomous vehicles based on the current trajectory and environment. This package `autoware_motion_velocity_planner_common` contains essential utilities and structures that support the planning process, including:

- Geometric operations for polygons and trajectories.
- General utilities for trajectory processing and visualization.
- Data structures for storing velocity planning results.

## Design

### Key Components

1. **Polygon Utilities (`polygon_utils.hpp`)**
   This component provides functions for handling geometric polygons related to motion planning. It includes:

   - Collision detection between trajectories and obstacles.
   - Creation of polygons representing the vehicle's trajectory at different time steps.
   - Geometric calculations using Boost Geometry.
2. **General Utilities (`utils.hpp`)**
   This component provides various utility functions, including:

   - Conversion between point representations (e.g., `pcl::PointXYZ` to `geometry_msgs::msg::Point`).
   - Distance calculations between trajectory points and obstacles.
   - Functions for concatenating vectors and processing trajectories.
   - Visualization tools for creating markers.
3. **Velocity Planning Results (`velocity_planning_result.hpp`)**
   This component defines data structures for storing the results of velocity planning, including:

   - `SlowdownInterval`: Represents a segment where the vehicle should slow down, with specified start and end points and velocity.
   - `VelocityPlanningResult`: Contains a collection of stop points, slowdown intervals, and optional velocity limits and clear commands.

## Usage

### Including the Package

To use this package in your project, you need to include it in your CMakeLists.txt:

```
find_package(autoware_motion_velocity_planner_common REQUIRED)
```

### Example Usage

Here's a simple example demonstrating the use of the utility functions:

```cpp

# include "autoware/motion\_velocity\_planner\_common/utils.hpp"

const auto decimated\_traj\_points = autoware::motion\_velocity\_planner::utils::decimate\_trajectory\_points\_from\_ego(
traj\_points, current\_pose, ego\_nearest\_dist\_threshold, ego\_nearest\_yaw\_threshold,
p.decimate\_trajectory\_step\_length, 0.0);
```

this example is from autoware\_universe/planning/motion\_velocity\_planner/autoware\_motion\_velocity\_obstacle\_cruise\_module/src/obstacle\_cruise\_module.cpp
