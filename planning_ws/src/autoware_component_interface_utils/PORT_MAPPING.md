# ROS2 to ROS1 port map

This package is a full-validation exception, not an M0 message conversion.

| Upstream surface | ROS1 render | Preserved or explicit contract |
| --- | --- | --- |
| `CMakeLists.txt`, `package.xml` | Catkin export/install | C++17 requirement is exported through `CFG_EXTRAS`; installed public headers are under global `include/`. |
| `rclcpp/interface.hpp`, `rclcpp.hpp` | explicit `ros::NodeHandle` adaptor | caller callback queue remains owned by the caller. |
| `service_client.hpp` | `ros::ServiceClient` plus one serial dispatch worker | readiness and timeout exceptions remain; roscpp has no cancellation. |
| `service_server.hpp` | typed `ros::ServiceServer` callback | `ServiceException` is converted only when the response has `status`; status-less responses propagate it as upstream does. |
| `topic_publisher.hpp`, `topic_subscription.hpp` | `ros::Publisher` / `ros::Subscriber` | queue depth and latch come from completed component-interface specs; polling is latest-value consume-once. |
| ROS2 `tier4_system_msgs::msg::ServiceLog` | local `ROS_DEBUG_STREAM` | the accepted ROS1 provider has no `ServiceLog` interface, so no wire type is fabricated. |

`test/ros1_wrapper_test.cpp` exercises service readiness, request/response,
timeout visibility, and polling topic consumption against a live ROS master.
