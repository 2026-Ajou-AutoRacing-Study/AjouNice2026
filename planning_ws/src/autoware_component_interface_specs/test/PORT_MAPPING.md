# ROS2 test replacement map

The original `test_control.cpp`, `test_localization.cpp`, `test_map.cpp`,
`test_perception.cpp`, `test_planning.cpp`, `test_system.cpp`, and
`test_vehicle.cpp` assert ROS2 `rclcpp::QoS` and `RMW_QOS_*` APIs. They are
not retained as dormant ROS2 sources in the ROS1 package.

Their depth/reliable/durability coverage is replaced by
`qos_contract_test.cpp`, which asserts all sixteen message QoS policies and
all twenty-two public interface names against `Ros1Qos`. The observable
durability behavior is separately exercised by `qos_latch_runtime_test.cpp`:
all seven `TransientLocal` interfaces must replay to a late subscriber and a
volatile `Trajectory` must not replay. The latter test uses an actual ROS1
master, publishers, subscribers, and wall-clock deadline.
