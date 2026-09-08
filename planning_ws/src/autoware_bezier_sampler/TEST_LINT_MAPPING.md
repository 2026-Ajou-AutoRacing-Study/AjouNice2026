# ROS2 test and lint mapping

The upstream `BUILD_TESTING` block has three ROS2-only test dependencies.

| Upstream dependency | ROS1 disposition | Executable verification |
| --- | --- | --- |
| `ament_cmake_ros` | `catkin` supplies `CATKIN_ENABLE_TESTING` and `catkin_add_gtest`. | `test_vector_layout` is registered with CTest and links the built library. |
| `ament_lint_auto` | Intentionally excluded: no equivalent automatic ament lint aggregator exists in the ROS1 package contract. | If `catkin_lint` is installed, the P2/P3 harness invokes it; otherwise CMake configure plus the explicit gtest are the declared substitute, not a claimed lint equivalence. |
| `autoware_lint_common` | Intentionally excluded: it is a ROS2 Autoware lint rules package with no qualified ROS1 provider. | The source/header byte-preservation check and `test_vector_layout` are required; no claim is made that Autoware's ROS2 lint rules executed. |

The gtest's expected midpoint is deliberately `(1.25, 1.75)`. It records the empirically observed upstream vector-constructor row/layout behavior; changing it to the conventional collinear midpoint is prohibited.
