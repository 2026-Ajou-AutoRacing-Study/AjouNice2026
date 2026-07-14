import rosbag

bag_path = "/home/ubuntu2004/AjouNice2026/rosbags/morai_local_valid_ego_sensors.bag"
topics = ["/sensor/lidar1", "/sensor/imu"]

last = {}
count = {t: 0 for t in topics}
back = {t: 0 for t in topics}

with rosbag.Bag(bag_path, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=topics):
        if not hasattr(msg, "header"):
            continue

        stamp = msg.header.stamp.to_sec()
        bag_t = t.to_sec()
        count[topic] += 1

        if topic in last and stamp < last[topic]:
            back[topic] += 1
            print(f"[BACK] {topic}: prev_header={last[topic]:.9f}, now_header={stamp:.9f}, bag_time={bag_t:.9f}, seq={msg.header.seq}")

        last[topic] = stamp

print()
print("summary")
for topic in topics:
    print(f"{topic}: count={count[topic]}, backward={back[topic]}")
