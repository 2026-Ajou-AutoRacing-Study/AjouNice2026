---
source_type: rendered_autoware_docs
site: autoware_core
title: Objects Of Interest Marker Interface
canonical_url: https://autowarefoundation.github.io/autoware_core/main/planning/autoware_objects_of_interest_marker_interface/
rendered_html: rendered/autoware_core_html/index-58.html
generated: 2026-06-24T15:00:04.823327+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Objects Of Interest Marker Interface

## Overview

`autoware_objects_of_interest_marker_interface` is a collection of object visualization function packages.

## Design

This package implement a library to manage and visualize the object information by construct and publish it as marker array to rviz.

For a object to be visualized, it has three import characteristics.

- pose the position of the object
- shape the shape of the Bounding box of the object
- color the color of the Bounding box of the object

## Usage

### init

include the header file to use then init the library

```cpp

# include

autoware::objects\_of\_interest\_marker\_interface::ObjectsOfInterestMarkerInterface
objects\_of\_interest\_marker\_interface\_{this, "obstacle\_cruise\_planner"};
```

### insert

insert object information to the 'objects\_of\_interest\_marker\_interface' manager

```
using autoware::objects_of_interest_marker_interface::ColorName;
objects_of_interest_marker_interface_.insertObjectData(
    stopped_obstacle.pose, stopped_obstacle.shape, ColorName::RED);
```

### publish

publish object information to the rviz to visualize

```
objects_of_interest_marker_interface_.publishMarkerArray();
```
