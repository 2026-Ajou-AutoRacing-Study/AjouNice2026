# Source-to-candidate map

| Upstream surface | ROS1 disabled-profile disposition |
| --- | --- |
| `message_filters.hpp` non-Agnocast branch | Preserved as native ROS1 `message_filters` aliases. |
| `CMakeLists.txt` non-Agnocast dependency branch | Replaced by Catkin export of `roscpp` and `message_filters`; only `0`/unset is accepted. |
| `autoware_agnocast_wrapper.hpp` | Omitted: all exposed macros are rclcpp/ROS2 ownership APIs; mapping them to ROS1 pointers would be a silent semantic rewrite. |
| `node.hpp`, `src/node.cpp` | Omitted: rclcpp interfaces, parameters, callback groups, and runtime node switching have no established package-independent ROS1 equivalent. |
| `autoware_agnocast_wrapper_register_node.cmake`, template | Omitted: ROS2 component/executor registration requires a downstream package-owned ROS1 executable/nodelet contract. |
| `agnocast_env.launch.*` | Omitted: it selects `agnocast_components` and injects a heaphook with `LD_PRELOAD`; neither is available or claimed here. |

The omission of the latter surfaces is intentional and is not Agnocast enabled
feature equivalence.

