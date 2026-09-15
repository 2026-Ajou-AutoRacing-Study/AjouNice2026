# MORAI UDP receipt-time policy

Live operation uses `/use_sim_time=false`. All supported MORAI UDP streams
use the ROS host clock. No fixed 110/122 ms correction is applied.

## Timestamp ownership audit

| Path | Policy |
|---|---|
| Ego / Competition / Object / Collision UDP | Capture `ros::Time::now()` once immediately after successful `recv`, before parsing; use for output header. Ignore remote packet sec/nsec for ROS timing. |
| IMU UDP | Same receipt-time policy. |
| GPS NMEA UDP | Capture time immediately after `recv`, before NMEA parsing. Previously already host time, but stamped after parsing. |
| Camera UDP | First received fragment of assembled frame supplies its timestamp. JPEG assembly and publication do not replace it. Remote fragment timestamps are not used. |
| LiDAR | Existing Velodyne host-time mode: `gps_time=false`, `pcap_time=false`, `time_offset=0`; scan header uses first packet. Scan accumulation still adds real latency. |
| `maptf_morai` / direct GT localization | Launch forwards to `localization_morai_gt`. Ego TF broadcasters preserve the upstream receipt-time stamp. Direct GT publisher queries TF at that Ego stamp and preserves it; no downstream restamping is applied. |
| `localization` / GNSS IMU GT CTRA | GPS odometry, IMU covariance adapter and GT motion adapter preserve input stamps. Local/global EKF consume inputs in one ROS clock domain. Kinematic publisher preserves EKF output header. |
| BEVFusion | Camera/LiDAR matching uses their headers. Detection publication keeps the input LiDAR stamp across asynchronous inference; never relabel detections with inference completion time. |
| Tracking / prediction | Measurement TF lookup and prediction remain in the input ROS clock domain. Intentional state extrapolation to a later timestamp is distinct from blind restamping. Map prediction preserves track header. |
| Control | Trajectory/state freshness checks remain enabled. `/ctrl_cmd` has no header; rosbag recording time is the available command time. |

Static TF is timeless and does not require periodic restamping. This change
applies to the common UDP receiver, hence both `udp` and `udp_lan`. Other
transports (rosbridge), old bags, and unrelated hardware drivers are not converted.

Receipt time is userspace dequeue time, not the kernel NIC arrival timestamp
and not the original measurement time. Transport, socket queuing and camera/scan
assembly latency remain. Camera/LiDAR physical synchronization must still be
evaluated; one clock domain does not imply simultaneous measurements.

## Validation and rollout

Build/install `sensor_interface`; then run:

```bash
source base_ws/install_aw149/setup.bash
python3 base_ws/src/sensor_interface/test/test_udp_receipt_time.py
```

The test uses an isolated ROS master, temporary loopback endpoints and installed
receiver binaries. It sends future/past remote stamps (+/-120 s) through Ego,
Competition, Object, Collision, IMU, GPS and camera paths and asserts every
output stamp is between send and ROS observation times. No production nodes
are started or stopped by the test.

Stop the vehicle before deployment. Restart UDP receivers, localization/TF/EKF,
perception and control to discard prior-clock messages in queues and TF caches.
Use only the selected localization provider, not direct GT and EKF simultaneously.
The existing processes retain old binaries until restarted.

Record the same `control_lan` bag and parameters after restart. Compare source
stamp age, raw Ego-to-EKF heading lag and oscillation at unchanged control gains.
Build and synthetic transport checks do not establish closed-loop stability.
