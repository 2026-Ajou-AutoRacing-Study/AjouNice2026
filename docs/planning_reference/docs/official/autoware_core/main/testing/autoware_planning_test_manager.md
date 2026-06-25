---
source_type: rendered_autoware_docs
site: autoware_core
title: Autoware Planning Test Manager
canonical_url: https://autowarefoundation.github.io/autoware_core/main/testing/autoware_planning_test_manager/
rendered_html: rendered/autoware_core_html/index-75.html
generated: 2026-06-24T15:00:05.258020+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Autoware Planning Test Manager

## Background

In each node of the planning module, when exceptional input, such as unusual routes or significantly deviated ego-position, is given, the node may not be prepared for such input and could crash. As a result, debugging node crashes can be time-consuming. For example, if an empty trajectory is given as input and it was not anticipated during implementation, the node might crash due to the unaddressed exceptional input when changes are merged, during scenario testing or while the system is running on an actual vehicle.

## Purpose

The purpose is to provide a utility for implementing tests to ensure that node operates correctly when receiving exceptional input. By utilizing this utility and implementing tests for exceptional input, the purpose is to reduce bugs that are only discovered when actually running the system, by requiring measures for exceptional input before merging PRs.

## Features

### Confirmation of normal operation

For the test target node, confirm that the node operates correctly and publishes the required messages for subsequent nodes. To do this, test\_node publish the necessary messages and confirm that the node's output is being published.

### Robustness confirmation for special inputs

After confirming normal operation, ensure that the test target node does not crash when given exceptional input. To do this, provide exceptional input from the test\_node and confirm that the node does not crash.

(WIP)

## Usage

```cpp

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
rclcpp::init(0, nullptr);

// instantiate test\_manager with PlanningInterfaceTestManager type
auto test\_manager = std::make\_shared();

// get package directories for necessary configuration files
const auto autoware\_test\_utils\_dir =
ament\_index\_cpp::get\_package\_share\_directory("autoware\_test\_utils");
const auto target\_node\_dir =
ament\_index\_cpp::get\_package\_share\_directory("target\_node");

// set arguments to get the config file
node\_options.arguments(
{"--ros-args", "--params-file",
autoware\_test\_utils\_dir + "/config/test\_vehicle\_info.param.yaml", "--params-file",
autoware\_planning\_validator\_dir + "/config/planning\_validator.param.yaml"});

// instantiate the TargetNode with node\_options
auto test\_target\_node = std::make\_shared(node\_options);

// publish the necessary topics from test\_manager second argument is topic name
test\_manager->publishOdometry(test\_target\_node, "/localization/kinematic\_state");
test\_manager->publishMaxVelocity(
test\_target\_node, "velocity\_smoother/input/external\_velocity\_limit\_mps");

// set scenario\_selector's input topic name(this topic is changed to test node)
test\_manager->setTrajectoryInputTopicName("input/parking/trajectory");

// test with normal trajectory
ASSERT\_NO\_THROW(test\_manager->testWithNominalTrajectory(test\_target\_node));

// make sure target\_node is running
EXPECT\_GE(test\_manager->getReceivedTopicNum(), 1);

// test with trajectory input with empty/one point/overlapping point
ASSERT\_NO\_THROW(test\_manager->testWithAbnormalTrajectory(test\_target\_node));

// shutdown ROS context
rclcpp::shutdown();
}
```

## Implemented tests

| Node | Test name | exceptional input | output | Exceptional input pattern |
| --- | --- | --- | --- | --- |
| autoware\_planning\_validator | NodeTestWithExceptionTrajectory | trajectory | trajectory | Empty, single point, path with duplicate points |
| velocity\_smoother | NodeTestWithExceptionTrajectory | trajectory | trajectory | Empty, single point, path with duplicate points |
| obstacle\_velocity\_limiter | NodeTestWithExceptionTrajectory | trajectory | trajectory | Empty, single point, path with duplicate points |
| path\_optimizer | NodeTestWithExceptionTrajectory | trajectory | trajectory | Empty, single point, path with duplicate points |
| scenario\_selector | NodeTestWithExceptionTrajectoryLaneDrivingMode NodeTestWithExceptionTrajectoryParkingMode | trajectory | scenario | Empty, single point, path with duplicate points for scenarios:LANEDRIVING and PARKING |
| freespace\_planner | NodeTestWithExceptionRoute | route | trajectory | Empty route |
| behavior\_path\_planner | NodeTestWithExceptionRoute NodeTestWithOffTrackEgoPose | route | route odometry | Empty route Off-lane ego-position |
| behavior\_velocity\_planner | NodeTestWithExceptionPathWithLaneID | path\_with\_lane\_id | path | Empty path |

## Important Notes

During test execution, when launching a node, parameters are loaded from the parameter file within each package. Therefore, when adding parameters, it is necessary to add the required parameters to the parameter file in the target node package. This is to prevent the node from being unable to launch if there are missing parameters when retrieving them from the parameter file during node launch.

## Future extensions / Unimplemented parts

(WIP)
