---
source_type: rendered_autoware_docs
site: autoware_universe
title: RTC Interface
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_rtc_interface/
rendered_html: rendered/autoware_universe_html/index-168.html
generated: 2026-06-24T14:59:32.433076+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# RTC Interface

## Purpose

RTC Interface is an interface to publish the decision status of behavior planning modules and receive execution command from external of an autonomous driving system.

## Inner-workings / Algorithms

The RTC Interface works by creating a communication channel between a behavior planning module and the rest of the autonomous driving system.
The planning module uses the interface to publish its current status, such as whether it is safe to proceed or if it is waiting for an external command.
Other modules can then subscribe to this status information and send commands to the behavior planning module, such as "activate" or "deactivate".

### Usage example

```c++
// Generate instance (in this example, "intersection" is selected)
autoware::rtc\_interface::RTCInterface rtc\_interface(node, "intersection");

// Generate UUID
const unique\_identifier\_msgs::msg::UUID uuid = generate\_uuid(getModuleId());

// Repeat while module is running
while (...) {
// Get safety status of the module corresponding to the module id
const bool safe = ...

// Get the current state of the module
const uint8\_t state = ...

// Get distance to the object corresponding to the module id
const double start\_distance = ...
const double finish\_distance = ...

// Get time stamp
const rclcpp::Time stamp = ...

// Update status
rtc\_interface.updateCooperateStatus(uuid, safe, state, start\_distance, finish\_distance, stamp);

if (rtc\_interface.isActivated(uuid)) {
// Execute planning
} else {
// Stop planning
}
// Get time stamp
const rclcpp::Time stamp = ...

// Publish status topic
rtc\_interface.publishCooperateStatus(stamp);
}

// Remove the status from array
rtc\_interface.removeCooperateStatus(uuid);
```

## Inputs / Outputs

### RTCInterface (Constructor)

```
autoware::rtc_interface::RTCInterface(rclcpp::Node & node, const std::string & name, const bool enable_rtc = false);
```

#### Description

A constructor for `autoware::rtc_interface::RTCInterface`.

#### Input

- `node` : Node calling this interface
- `name` : Name of cooperate status array topic and cooperate commands service
  - Cooperate status array topic name : `~/{name}/cooperate_status`
  - Cooperate commands service name : `~/{name}/cooperate_commands`
- `enable_rtc`: A boolean indicating whether RTC is enabled or not by default.

#### Output

An instance of `RTCInterface`

### publishCooperateStatus

```
autoware::rtc_interface::publishCooperateStatus(const rclcpp::Time & stamp)
```

#### Description

Publish registered cooperate status.

#### Input

- `stamp` : Time stamp

#### Output

Nothing

### updateCooperateStatus

```
autoware::rtc_interface::updateCooperateStatus(const unique_identifier_msgs::msg::UUID & uuid, const bool safe, const double start_distance, const double finish_distance, const rclcpp::Time & stamp)
```

#### Description

Update cooperate status corresponding to `uuid`.
If cooperate status corresponding to `uuid` is not registered yet, add new cooperate status.

#### Input

- `uuid` : UUID for requesting module
- `safe` : Safety status of requesting module
- `start_distance` : Distance to the start object from ego vehicle
- `finish_distance` : Distance to the finish object from ego vehicle
- `stamp` : Time stamp
- `requested`: A boolean indicating whether a request has been made.
- `override_rtc_auto_mode`: An optional boolean to override the RTC mode (true forces AUTO, false forces MANUAL).

#### Output

Nothing

### removeCooperateStatus

```
autoware::rtc_interface::removeCooperateStatus(const unique_identifier_msgs::msg::UUID & uuid)
```

#### Description

Remove cooperate status corresponding to `uuid` from registered statuses.

#### Input

- `uuid` : UUID for expired module

#### Output

Nothing

### clearCooperateStatus

```
autoware::rtc_interface::clearCooperateStatus()
```

#### Description

Remove all cooperate statuses.

#### Input

Nothing

#### Output

Nothing

### isActivated

```
autoware::rtc_interface::isActivated(const unique_identifier_msgs::msg::UUID & uuid)
```

#### Description

Return received command status corresponding to `uuid`.

#### Input

- `uuid` : UUID for checking module

#### Output

If auto mode is enabled, return based on the safety status.
If not, if received command is `ACTIVATED`, return `true`.
If not, return `false`.

### isRegistered

```
autoware::rtc_interface::isRegistered(const unique_identifier_msgs::msg::UUID & uuid)
```

#### Description

Return `true` if `uuid` is registered.

#### Input

- `uuid` : UUID for checking module

#### Output

If `uuid` is registered, return `true`.
If not, return `false`.

## Debugging Tools

There is a debugging tool called [RTC Replayer](https://autowarefoundation.github.io/autoware_tools/main/planning/autoware_rtc_replayer/) for the RTC interface.

## Assumptions / Known limits

## Future extensions / Unimplemented parts
