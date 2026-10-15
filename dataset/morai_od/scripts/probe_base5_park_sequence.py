#!/usr/bin/env python3
"""Stationary gear-only Park repetition probe; no scene loads, captures or Drive."""
import argparse
import math
from pathlib import Path
import threading
import time
from base5_capture_safety import atomic_json,park_request_fields
from run_base5_capture import bounded_call,message_dict


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--count',type=int,default=12)
    args=parser.parse_args()
    if not 1<=args.count<=24:parser.error('Bounded 1–24 requests only')
    if args.output.exists():raise FileExistsError('Evidence overwrite refused')
    import rospy
    import rosgraph
    from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
    from morai_msgs.srv import MoraiEventCmdSrv,MoraiEventCmdSrvRequest
    rospy.init_node('base5_park_sequence_probe',anonymous=True,disable_signals=True)
    latest={};samples=[];halt=threading.Event()
    def callback(msg):
        latest.update(t=time.monotonic(),message=message_dict(msg));samples.append(dict(latest))
    subscriber=rospy.Subscriber('/Ego_topic',EgoVehicleStatus,callback,queue_size=100)
    control=rospy.Publisher('/ctrl_cmd',CtrlCmd,queue_size=1)
    report=dict(scene_load=False,capture=False,drive_request=False,requests=[],passed=False)
    def stationary():
        if not latest or time.monotonic()-latest['t']>1:raise RuntimeError('Ego stale')
        velocity=latest['message']['velocity']
        if math.hypot(velocity['x'],velocity['y'])>.02:raise RuntimeError('Ego not stationary')
        if not control.get_num_connections():raise RuntimeError('Brake subscriber absent')
    def brake():
        while not halt.is_set():
            control.publish(CtrlCmd(longlCmdType=1,brake=1.,accel=0.,steering=0.));time.sleep(.025)
    try:
        deadline=time.monotonic()+4
        while time.monotonic()<deadline and (not latest or not control.get_num_connections()):time.sleep(.025)
        stationary();threading.Thread(target=brake,daemon=True).start();time.sleep(.3)
        rospy.wait_for_service('/Service_MoraiEventCmd',timeout=3.)
        for index in range(args.count):
            gap=(0.,.05,.25)[index%3];time.sleep(gap);stationary()
            request=MoraiEventCmdSrvRequest()
            for key,value in park_request_fields().items():setattr(request.request,key,value)
            proxy=rospy.ServiceProxy('/Service_MoraiEventCmd',MoraiEventCmdSrv)
            row=dict(index=index,gap_sec=gap,request=message_dict(request),
                service_uri=rosgraph.Master(rospy.get_name()).lookupService('/Service_MoraiEventCmd'))
            started=time.monotonic()
            try:
                response=bounded_call(lambda:proxy(request),timeout=3.)
                row.update(gear=response.response.gear,park_verified=response.response.gear==1)
                if not row['park_verified']:raise RuntimeError('Park not confirmed')
            except Exception as exc:
                row['error']=str(exc);row['park_verified']=False
                raise
            finally:
                row['elapsed_sec']=time.monotonic()-started
                report['requests'].append(row);atomic_json(args.output,report)
                print(row,flush=True)
        report['passed']=True
    except BaseException as exc:report['error']=str(exc) or type(exc).__name__
    finally:
        report.update(finished_at=time.time(),ego_samples=samples)
        atomic_json(args.output,report);halt.set();subscriber.unregister()
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
