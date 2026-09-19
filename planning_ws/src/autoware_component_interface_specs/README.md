# autoware_component_interface_specs

This package defines the standardized component interface specifications for Autoware Core, ensuring consistent communication and interaction between various components in the Autoware autonomous driving stack.

## Purpose

The purpose of this package is to:

- Provide a single source of truth for component interface definitions
- Ensure consistency across different implementations
- Facilitate modular development and component interchangeability
- Document the communication protocols between Autoware Core components

## Structure

The package contains interface specifications for various components, including:

- Message definitions
- Service interfaces
- Action interfaces

## Usage

To use these interface specifications in your component:

1. Add this package as a dependency in your package.xml:

   ```xml
   <depend>autoware_component_interface_specs</depend>
   ```

2. Use the provided interfaces in your component code.

   ```cpp
   #include <autoware/component_interface_specs/localization.hpp>
   // ROS1 publisher creation using the explicit queue/latch policy.
   using KinematicState = autoware::component_interface_specs::localization::KinematicState;
   const auto qos = autoware::component_interface_specs::get_qos<KinematicState>();
   auto publisher = node_handle.advertise<KinematicState::Message>(
     KinematicState::name, qos.queue_size, qos.latch);
   auto subscriber = node_handle.subscribe<KinematicState::Message>(
     KinematicState::name, qos.queue_size, callback);
   ```

`Ros1Qos::latch` is the explicit ROS1 replacement for the ROS2
`TransientLocal` contract. `Ros1Qos::reliability` preserves the original
reliable intent for review, but TCPROS has no DDS reliability negotiation or
history-policy equivalent.
