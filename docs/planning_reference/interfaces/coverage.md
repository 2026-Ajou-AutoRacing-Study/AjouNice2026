> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Planning Interface Coverage

This file explains what planning interface information is retained after pruning the reference corpus to planning-only content.

- Raw planning type mentions from `contracts.csv`: 128
- Autoware msg/srv/action types retained in `messages.csv`: 150
- Standard/external or helper type mentions not expanded locally: 31

## Retained Interface Files

- `contracts.csv`: topics, services, parameters, and evidence paths extracted from planning packages.
- `../modules/runtime_surfaces.csv`: planning components, plugins, and executables.
- `messages.csv`: Autoware message/service/action definitions used by planning interfaces.

## External Or Helper Type Mentions

These are mentioned by planning contracts but are standard ROS, visualization, utility, or generated helper types rather than Autoware message definitions retained in the local crosswalk.

- `AccelWithCovarianceStamped`
- `DiagnosticArray`
- `HADMapBin`
- `Imu`
- `MarkerArray`
- `MetricArrayMsg`
- `OccupancyGrid`
- `Odometry`
- `Odometry, autoware_utils::polling_policy::All`
- `PointCloud2`
- `PolygonStamped`
- `PoseArray`
- `PoseStamped`
- `SetBool`
- `TwistStamped`
- `geometry_msgs::msg::AccelWithCovarianceStamped`
- `geometry_msgs::msg::PoseStamped`
- `geometry_msgs::msg::PoseWithCovarianceStamped`
- `nav_msgs::msg::OccupancyGrid`
- `nav_msgs::msg::Odometry`
- `sensor_msgs::msg::PointCloud2`
- `std_msgs::msg::Bool`
- `std_msgs::msg::Empty`
- `std_msgs::msg::Float32`
- `std_msgs::msg::Float32MultiArray`
- `std_msgs::msg::Float64`
- `std_msgs::msg::Float64MultiArray`
- `std_msgs::msg::Int32`
- `std_msgs::msg::String`
- `tf2_msgs::msg::TFMessage`
- `visualization_msgs::msg::MarkerArray`
