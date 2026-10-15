#!/usr/bin/env python3
"""Single bounded stationary Park request; no scene load, capture or throttle."""
import argparse
import math
from pathlib import Path
import threading
import time
from base5_capture_safety import atomic_json
from run_base5_capture import bounded_call,message_dict


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--option',type=int,choices=(2,3),default=2)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError('Probe evidence overwrite refused')
    import rospy
    from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
    from morai_msgs.srv import MoraiEventCmdSrv,MoraiEventCmdSrvRequest
    rospy.init_node('base5_stationary_park_probe',anonymous=True,disable_signals=True)
    samples=[];halt=threading.Event()
    def callback(msg):samples.append(dict(t=time.monotonic(),message=message_dict(msg)))
    subscriber=rospy.Subscriber('/Ego_topic',EgoVehicleStatus,callback,queue_size=100)
    control=rospy.Publisher('/ctrl_cmd',CtrlCmd,queue_size=1)
    report=dict(option=a.option,requested_gear=1,scene_load=False,capture=False,park_verified=False)
    try:
        deadline=time.monotonic()+4
        while time.monotonic()<deadline and (not samples or not control.get_num_connections()):time.sleep(.05)
        if not samples or time.monotonic()-samples[-1]['t']>1:raise RuntimeError('Ego connection stale')
        velocity=samples[-1]['message']['velocity']
        if math.hypot(velocity['x'],velocity['y'])>.02:raise RuntimeError('Ego not stationary; no gear request sent')
        if not control.get_num_connections():raise RuntimeError('Brake subscriber absent')
        def brake():
            while not halt.is_set():
                control.publish(CtrlCmd(longlCmdType=1,brake=1.,accel=0.,steering=0.));time.sleep(.025)
        thread=threading.Thread(target=brake,daemon=True);thread.start()
        time.sleep(.25)
        rospy.wait_for_service('/Service_MoraiEventCmd',timeout=3.)
        request=MoraiEventCmdSrvRequest();request.request.option=a.option
        request.request.ctrl_mode=3;request.request.gear=1
        report['request']=message_dict(request);started=time.monotonic()
        try:
            response=bounded_call(lambda:rospy.ServiceProxy('/Service_MoraiEventCmd',MoraiEventCmdSrv)(request),timeout=3.)
            report.update(response=message_dict(response),park_verified=response.response.gear==1)
        finally:report['request_elapsed_sec']=time.monotonic()-started
        time.sleep(.25)
    except Exception as exc:report['error']=str(exc)
    finally:
        report['ego_samples']=samples;report['finished_at']=time.time()
        atomic_json(a.output,report);halt.set()
        subscriber.unregister()
    print({k:v for k,v in report.items() if k!='ego_samples'},flush=True)
    return 0 if report['park_verified'] else 1


if __name__=='__main__':raise SystemExit(main())
