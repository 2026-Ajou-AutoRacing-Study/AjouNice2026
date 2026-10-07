#!/usr/bin/env python3
"""Explicit live input fault injection. Never run with production/data campaigns."""
import argparse
import copy
import json
import time
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--campaign',type=Path,required=True)
    p.add_argument('--approve-test-injection',action='store_true',required=True);a=p.parse_args()
    import rospy
    from morai_msgs.msg import CollisionData,ObjectStatusList
    rospy.init_node('base5_explicit_collision_input_test',anonymous=True)
    child=a.campaign/'chunk_0000/capture';deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        try:
            state=json.loads((child/'status.json').read_text())
            if state.get('stage')=='scene_load':break
        except FileNotFoundError:pass
        time.sleep(.025)
    else:raise RuntimeError('No first diagnostic load; injected nothing')
    manifest=json.loads((a.campaign/'chunk_0000/manifest.json').read_text())
    meta=json.loads(Path(manifest['scenes'][0]['metadata']).read_text())
    if not(meta.get('calibration_only') and meta.get('collision_skip_diagnostic')):
        raise ValueError('Fault injection refused for non-diagnostic scene')
    # Only after the new normal scene has begun publishing; preserve actual Ego pose.
    time.sleep(.85)
    collision=rospy.wait_for_message('/CollisionData',CollisionData,timeout=3)
    objects=rospy.wait_for_message('/Object_topic',ObjectStatusList,timeout=3)
    if not objects.npc_list:raise RuntimeError('No peer fixture; injected nothing')
    peer=copy.deepcopy(objects.npc_list[0]);peer.name='INJECTED_TEST_CONTACT'
    collision.collision_object.append(peer)
    pub=rospy.Publisher('/CollisionData',CollisionData,queue_size=10)
    until=time.monotonic()+3
    while not pub.get_num_connections() and time.monotonic()<until:time.sleep(.025)
    if not pub.get_num_connections():raise RuntimeError('No test subscriber; injected nothing')
    for _ in range(5):pub.publish(collision);time.sleep(.025)
    print('Injected 5 labelled test contact messages; not a physical-collision certification',flush=True)

if __name__=='__main__':main()
