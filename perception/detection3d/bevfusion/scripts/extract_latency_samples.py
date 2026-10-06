#!/usr/bin/env python3
"""Keep a few untouched ROS LiDAR/nearest compressed-camera samples for timing."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import rosbag
import rospy


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bag',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    repo=Path(__file__).resolve().parents[4]
    sys.path.insert(0,str(repo/'base_ws/src/perception/bevfusion_ros/scripts'))
    from bevfusion_ros_bridge import pointcloud_to_xyzi
    cameras={f'/sensor/camera/{key}/compressed':'camera_'+key for key in ('front','front_left','front_right','back')}
    a.output.mkdir(parents=True,exist_ok=False);report=[]
    with rosbag.Bag(str(a.bag)) as bag:
        duration=bag.get_end_time()-bag.get_start_time()
        for index,fraction in enumerate((.2,.5,.8)):
            target=bag.get_start_time()+duration*fraction
            clouds=[];images={k:[] for k in cameras}
            for topic,msg,t in bag.read_messages(topics=['/sensor/lidar1']+list(cameras),
                    start_time=rospy.Time.from_sec(target-.5),end_time=rospy.Time.from_sec(target+.5)):
                if topic=='/sensor/lidar1':clouds.append((t.to_sec(),msg))
                else:images[topic].append(msg)
            _,cloud=min(clouds,key=lambda item:abs(item[0]-target));stamp=cloud.header.stamp.to_sec()
            payload={'points':pointcloud_to_xyzi(cloud)};diffs={}
            for topic,frame in cameras.items():
                image=min(images[topic],key=lambda m:abs(m.header.stamp.to_sec()-stamp))
                diffs[frame]=abs(image.header.stamp.to_sec()-stamp)
                if diffs[frame]>.10:raise ValueError('Incomplete matched sample')
                payload[frame]=np.frombuffer(image.data,dtype=np.uint8)
            np.savez(a.output/f'sample_{index}.npz',**payload)
            report.append(dict(index=index,stamp=stamp,points=len(payload['points']),camera_dt=diffs))
    (a.output/'samples.json').write_text(json.dumps(dict(bag=str(a.bag.resolve()),samples=report),indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':main()
