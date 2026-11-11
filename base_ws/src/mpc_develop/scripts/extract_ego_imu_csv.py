#!/usr/bin/env python3
"""Export original ROS bag Ego and IMU samples for reference-point diagnosis."""
import argparse
import csv
from pathlib import Path
import rosbag

def timestamp(msg, bag_time):
    stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
    return stamp.to_sec() if stamp is not None and stamp.to_sec() > 0 else bag_time.to_sec()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    with rosbag.Bag(args.bag) as bag, (out / 'ego.csv').open('x', newline='') as ef, (out / 'imu.csv').open('x', newline='') as imf:
        e, i = csv.writer(ef), csv.writer(imf)
        e.writerow(['t','pos_x','pos_y','heading','vel_x','vel_y','acc_x','acc_y','wheel_angle'])
        i.writerow(['t','wx','wy','wz','ax','ay','az'])
        counts = [0, 0]
        for topic, msg, bt in bag.read_messages(topics=['/Ego_topic', '/sensor/imu']):
            t = timestamp(msg, bt)
            if topic == '/Ego_topic':
                e.writerow([t,msg.position.x,msg.position.y,msg.heading,msg.velocity.x,msg.velocity.y,msg.acceleration.x,msg.acceleration.y,msg.wheel_angle]); counts[0] += 1
            else:
                i.writerow([t,msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]); counts[1] += 1
    print('Ego samples: {}; IMU samples: {}'.format(*counts))
    if min(counts) == 0:
        raise RuntimeError('Required topic missing; exported files are incomplete')

if __name__ == '__main__':
    main()
