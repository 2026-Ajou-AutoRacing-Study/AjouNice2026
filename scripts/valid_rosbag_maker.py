import rosbag

in_path = "/home/ubuntu2004/AjouNice2026/rosbags/morai_local_valid_ego_sensors.bag"
out_path = "/home/ubuntu2004/AjouNice2026/rosbags/morai_local_valid_ego_sensors_fixed_time.bag"

fix_header_topics = {
    "/lidar3D",
    "/imu",
}

with rosbag.Bag(out_path, "w") as outbag:
    with rosbag.Bag(in_path, "r") as inbag:
        for topic, msg, t in inbag.read_messages():
            # LiDAR/IMU header stamp를 rosbag 기록 시간으로 교체.
            # FAST-LIO offline 테스트용.
            if topic in fix_header_topics and hasattr(msg, "header"):
                msg.header.stamp = t

            # TF도 같이 bag time으로 맞춤.
            if topic == "/tf":
                for tr in msg.transforms:
                    tr.header.stamp = t

            outbag.write(topic, msg, t)

print(f"wrote: {out_path}")