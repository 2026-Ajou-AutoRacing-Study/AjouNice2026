# RTC Interface ROS1 Port

## Scope

`rtc_interface_ros1.py` ports the Autoware Request To Cooperate communication
surface to ROS1. It is shared infrastructure, while Lane Change is the first
BPP module connected to it in both execution styles:

- standalone `lane_change_bpp.py`
- integrated `behavior_path_planner.py`

The debug `std_msgs/String` status remains available for existing tools, but
RTC decisions now use typed messages and services.

## ROS1 Contracts

| Direction | Interface | Type |
| --- | --- | --- |
| Left | `/planning/cooperate_status/lane_change_left` | `tier4_rtc_msgs/CooperateStatusArray` |
| Left | `/planning/cooperate_commands/lane_change_left` | `tier4_rtc_msgs/CooperateCommands` service |
| Left | `/planning/enable_auto_mode/lane_change_left` | `tier4_rtc_msgs/AutoMode` service |
| Right | `/planning/cooperate_status/lane_change_right` | `tier4_rtc_msgs/CooperateStatusArray` |
| Right | `/planning/cooperate_commands/lane_change_right` | `tier4_rtc_msgs/CooperateCommands` service |
| Right | `/planning/enable_auto_mode/lane_change_right` | `tier4_rtc_msgs/AutoMode` service |

Each lane-change request receives a UUID. The interface keeps its safety,
command, state, mode, and nominal start/finish distances until the request is
completed or replaced.

## Path Gate

| Mode | Safety | Command | Final output |
| --- | --- | --- | --- |
| Auto | Safe | Ignored | Candidate path, `RUNNING` |
| Auto | Unsafe | Ignored | Reference path, `WAITING_FOR_EXECUTION` |
| Manual | Any | `DEACTIVATE` | Reference path, `WAITING_FOR_EXECUTION` |
| Manual | Any | `ACTIVATE` | Candidate path, `RUNNING` |

Changing an activated request to deactivated produces `ABORTING` for one
planning cycle before returning to `WAITING_FOR_EXECUTION`. A request that
finishes after running publishes `SUCCEEDED` before it is removed.

The proposal and root paths use the Autoware-compatible topics:

```text
/planning/path_candidate/lane_change_left
/planning/path_reference/lane_change_left
/planning/path_candidate/lane_change_right
/planning/path_reference/lane_change_right
```

The existing ROS1 final path topics remain unchanged.

## Mode Selection

`enable_rtc=false` is the default and starts the interface in Auto mode, which
preserves the previous ROS1 behavior. Set the launch argument to `true` to
start in Manual mode:

```bash
roslaunch behavior_path_planner behavior_path_planner.launch \
  lane_change_enable_rtc_left:=true
```

Mode can also be changed while running:

```bash
rosservice call /planning/enable_auto_mode/lane_change_left "enable: false"
```

Read the active UUID from the status topic, then send a command with the same
UUID and module type `1` (`LANE_CHANGE_LEFT`):

```bash
rostopic echo /planning/cooperate_status/lane_change_left

rosservice call /planning/cooperate_commands/lane_change_left "
stamp: {secs: 0, nsecs: 0}
commands:
- uuid: {uuid: [UUID_BYTES_HERE]}
  module: {type: 1}
  command: {type: 1}
"
```

Command type `1` is `ACTIVATE`; type `0` is `DEACTIVATE`.

## Verified Behavior

The K-City left lane-change request was tested through the standalone and
integrated BPP nodes.

```text
Auto + safe:                 RUNNING, candidate selected
Manual + no approval:        WAITING, reference selected
Manual + ACTIVATE:           RUNNING, candidate selected
Manual + DEACTIVATE:         ABORTING then WAITING, reference selected
Auto + unsafe object:        WAITING, reference selected
Candidate still published:   yes
Normal differential test:    PASS
Blocked differential test:   PASS
```

## Remaining Boundary

The common interface can be reused by other modules, but only Lane Change is
connected in this step. A central AD API cooperation aggregator and the ROS2
RTC Manager RViz plugin are not ported. Static avoidance, Start Planner, Goal
Planner, and other scene modules still need their own module-specific RTC
gating and lifecycle integration.
