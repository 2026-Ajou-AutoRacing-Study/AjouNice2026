# Dynamic obstacle avoidance candidate P3 runtime receipt

Scope: `candidate-r001` only.  This is candidate evidence and does not alter
the official AW149 count.

## ABI closure

`build-r002` was configured against the explicit ROS1 staging providers used
by the behavior-path-planner r025 ABI boundary, then built successfully.  The
result contains the candidate shared library and the three narrow runtime
probes under `devel/lib/autoware_behavior_path_dynamic_obstacle_avoidance_module/`.

## Foreground ROS runtime gate

On a scoped temporary ROS master (port 11492; stopped by the runner trap),
with only the explicit candidate and provider prefix paths:

1. `aw149_doa_manager_init` passed.  With no parameters it reported the
   missing typed parameters and returned success only because the manager
   rejected initialization (fail-closed assertion).
2. `rosparam load config/dynamic_obstacle_avoidance.ros1.yaml
   /aw149_doa_manager_config_init` followed by
   `aw149_doa_manager_config_init` passed, proving the translated typed
   parameter surface is consumable.
3. `aw149_doa_plugin_discovery` passed and emitted
   `AW149_DOA_PLUGIN_DISCOVERY_PASS`, proving pluginlib can instantiate the
   manager through the BPP scene-manager interface.

The three gate markers were:

```
P3_DOA_MANAGER_FAIL_CLOSED_RUNTIME_PASS
P3_DOA_MANAGER_CONFIG_RUNTIME_PASS
P3_DOA_PLUGIN_DISCOVERY_RUNTIME_PASS
```

## Boundary

This does not exercise dynamic-object inputs, path modification, nor a full
planner node.  It is therefore P3 candidate closure only, pending integrated
Behavior Path Planner runtime evidence and independent validation before any
official promotion.
