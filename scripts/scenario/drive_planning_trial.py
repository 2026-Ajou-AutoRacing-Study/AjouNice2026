#!/usr/bin/env python3
"""Own one simulated closed-loop drive and stop at the explicit Route B waypoint.

Requires a ready full stack and no existing /ctrl_cmd publisher. Not a resetter.
"""
import argparse
import json
import math
import signal
import subprocess
import sys
import time
import threading
from pathlib import Path
import rospy
from morai_msgs.msg import CollisionData, CtrlCmd, EgoVehicleStatus, ObjectStatusList
from autoware_planning_msgs.msg import Trajectory, LaneletRoute
from std_msgs.msg import String
from evaluate_planning_trial import footprint, vehicle_bbox_center_offset
from collision_review import PendingCollisionReview
from planner_failure_review import parse_failure_event
from trial_command_gate import TrialCommandGate


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--seconds', type=float, default=150)
    a = p.parse_args()
    rospy.init_node('planning_trial_driver', anonymous=True, disable_signals=True)
    interrupted = [False]
    def interrupt(_signum, _frame):
        interrupted[0] = True
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    publishers = rospy.get_master().getSystemState()[2][0]
    if any(topic == '/ctrl_cmd' and nodes for topic, nodes in publishers):
        raise RuntimeError('another /ctrl_cmd publisher exists')
    traj = rospy.wait_for_message('/planning/trajectory', Trajectory, timeout=10)
    if len(traj.points) < 3 or (rospy.Time.now()-traj.header.stamp).to_sec() > 0.5:
        raise RuntimeError('no fresh nondegenerate trajectory')
    state = dict(received=0., endpoint=False, speed=0., xy=None, ego=None,
                 contact_candidate=False, contact_evidence=None,
                 overlap_streak=0, objects={}, gt_received=0., route_uuid='', planner_failure=None)
    lock = threading.RLock()
    pending = PendingCollisionReview()
    def serialized(callback):
        def wrapped(*args):
            with lock:
                return callback(*args)
        return wrapped

    @serialized
    def on_route(message):
        state['route_uuid'] = bytes(message.uuid.uuid).hex()

    @serialized
    def on_planner_failure(message):
        event = parse_failure_event(message.data, rospy.Time.now().to_sec(), state['route_uuid'])
        if event is not None and state['planner_failure'] is None:
            state['planner_failure'] = event

    rospy.Subscriber('/planning/mission_planning/route', LaneletRoute, on_route, queue_size=1)
    rospy.Subscriber('/planning/debug/lane_change_safety_failure', String,
                     on_planner_failure, queue_size=10)

    def resolve_pending():
        if not pending.pending:
            return
        now = time.monotonic()
        objects = {}
        for object_id, obj in state['objects'].items():
            values = (obj.position.x, obj.position.y, obj.heading, obj.size.x, obj.size.y, obj.size.z,
                      obj.velocity.x, obj.velocity.y)
            objects[object_id] = dict(xy=(obj.position.x, obj.position.y),
                speed=math.hypot(obj.velocity.x, obj.velocity.y), message=obj,
                valid=all(math.isfinite(v) for v in values) and obj.size.x > 0 and obj.size.y > 0)
        def gap(event, candidate):
            ego, first = state['ego'], event['context']['ego_snapshot']
            if ego is None or now-state['received'] > .2 or first is None:
                return None
            npc = candidate['message']
            boxes = [footprint(e.position.x,e.position.y,e.heading,4.635,1.892,1.5275)
                     for e in (first, ego)]
            # Conservative swept ego region during the short association window.
            ep = boxes[0].union(boxes[1]).convex_hull
            np = footprint(npc.position.x,npc.position.y,npc.heading,npc.size.x,npc.size.y,
                           vehicle_bbox_center_offset(npc.size.x,npc.size.y,npc.size.z))
            return ep.distance(np)
        for outcome in pending.resolve(now, objects, state['gt_received'], gap):
            if outcome['review'] and not state['contact_candidate']:
                event = outcome['event']
                evidence = {k:v for k,v in event['context'].items() if k != 'ego_snapshot'}
                evidence.update({k:v for k,v in outcome.items() if k != 'event'})
                evidence.update(object_id=event['object_id'], reported_xy=event['xy'],
                                gt_age_sec=now-state['gt_received'])
                state['contact_evidence'] = evidence
                state['contact_candidate'] = True

    @serialized
    def on_ego(m):
        state['received'] = time.monotonic()
        state['speed'] = abs(m.velocity.x)
        state['xy'] = (m.position.x, m.position.y)
        state['ego'] = m
        state['endpoint'] = math.hypot(m.position.x+131.559130220-202.157329135,
            m.position.y+545.911338186-132.337284715) < 10
    rospy.Subscriber('/Ego_topic', EgoVehicleStatus, on_ego, queue_size=1)
    @serialized
    def on_collision(m):
        xy = state['xy']
        if not xy:
            return
        for o in m.collision_object:
            if math.hypot(o.position.x-xy[0], o.position.y-xy[1]) >= 15.:
                continue
            pending.report(o.unique_id, (o.position.x,o.position.y), time.monotonic(),
                dict(ros_time=rospy.Time.now().to_sec(), ego_xy=xy,
                     ego_snapshot=state['ego'], object_available=o.unique_id in state['objects']))
        resolve_pending()
    rospy.Subscriber('/CollisionData', CollisionData, on_collision, queue_size=1)
    @serialized
    def on_objects(m):
        state['objects'] = {o.unique_id:o for o in m.npc_list}
        state['gt_received'] = time.monotonic()
        resolve_pending()
        ego = state['ego']
        if ego is None or time.monotonic()-state['received'] > .2:
            return
        ep = footprint(ego.position.x, ego.position.y, ego.heading, 4.635, 1.892, 1.5275)
        overlaps = any(ep.intersects(footprint(o.position.x, o.position.y, o.heading,
                       o.size.x, o.size.y, vehicle_bbox_center_offset(o.size.x, o.size.y, o.size.z)))
                       for o in m.npc_list if o.size.x > 0 and o.size.y > 0 and
                       math.hypot(o.position.x-ego.position.x, o.position.y-ego.position.y) < 25.)
        state['overlap_streak'] = state['overlap_streak']+1 if overlaps else 0
        if state['overlap_streak'] >= 3:
            if not state['contact_candidate']:
                state['contact_evidence'] = dict(reason='gt_three_overlap_samples',
                    ros_time=rospy.Time.now().to_sec(), ego_xy=state['xy'])
            state['contact_candidate'] = True
    rospy.Subscriber('/Object_topic', ObjectStatusList, on_objects, queue_size=1)
    watchdog_stop = threading.Event()
    def watchdog():
        while not watchdog_stop.wait(.01):
            with lock:
                resolve_pending()
    threading.Thread(target=watchdog, name='collision_association_deadline', daemon=True).start()
    observe = subprocess.Popen([sys.executable, str(Path(__file__).with_name('observe_planning_trial.py')),
        '--output', str(a.output), '--seconds', str(a.seconds+40)])
    deadline = time.monotonic()+10
    while not a.output.exists() and time.monotonic() < deadline:
        if observe.poll() is not None:
            raise RuntimeError('observer exited')
        time.sleep(0.1)
    if not a.output.exists():
        stop(observe)
        raise RuntimeError('observer did not initialize')
    command_publisher = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
    intervention_publisher = rospy.Publisher('/evaluation/trial/intervention', String,
                                            queue_size=1, latch=True)
    command_gate = TrialCommandGate(command_publisher.publish, CtrlCmd)
    command_subscriber = rospy.Subscriber('/control/scenario_trial/raw_cmd', CtrlCmd,
                                          command_gate.receive, queue_size=1)
    brake_timer = rospy.Timer(rospy.Duration(.02), lambda _: command_gate.tick())
    connection_deadline = time.monotonic()+5
    while command_publisher.get_num_connections() == 0 and time.monotonic() < connection_deadline:
        time.sleep(.02)
    if command_publisher.get_num_connections() == 0:
        stop(observe)
        raise RuntimeError('trial command publisher has no simulator subscriber')
    with (a.output/'control.log').open('w') as logfile:
        controller = subprocess.Popen(['roslaunch', str(Path(__file__).with_name('trial_controller.launch'))], stdout=logfile,
            stderr=subprocess.STDOUT)
        started = time.monotonic()
        last_print = started
        reason = 'timeout'
        try:
            while not rospy.is_shutdown() and time.monotonic()-started < a.seconds:
                elapsed = time.monotonic()-started
                if interrupted[0]:
                    reason = 'interrupted'; break
                if state['planner_failure'] is not None:
                    reason = 'planner_no_safe_maneuver'; break
                if state['contact_candidate']:
                    reason = 'near_collision_review'; break
                if controller.poll() is not None or observe.poll() is not None:
                    reason = 'owned_process_exited'; break
                if elapsed > 5 and time.monotonic()-state['received'] > 1:
                    reason = 'ego_feedback_stale'; break
                if state['endpoint']:
                    reason = 'evaluation_waypoint_reached'; break
                if time.monotonic()-last_print > 10:
                    print('DRIVE', round(elapsed,1),
                          {k:v for k,v in state.items() if k not in ('ego', 'objects')}, flush=True)
                    last_print = time.monotonic()
                time.sleep(0.01)
        finally:
            watchdog_stop.set()
            # Latch before any blocking shutdown. Late controller messages can
            # update steering but can never overwrite emergency longitudinal commands.
            intervention = dict(stamp=rospy.Time.now().to_sec(), reason=reason,
                                action='latched_brake_controller_steering')
            command_gate.stop()
            intervention['first_brake_publish_completed_stamp'] = rospy.Time.now().to_sec()
            intervention_publisher.publish(String(data=json.dumps(intervention)))
            (a.output/'intervention.json').write_text(json.dumps(intervention, indent=2))
            brake_started = time.monotonic()
            stopped_confirmed = False
            while not rospy.is_shutdown() and time.monotonic()-brake_started < 20:
                with lock:
                    stopped_confirmed = (time.monotonic()-state['received'] < .5 and state['speed'] < .1)
                if time.monotonic()-brake_started >= 8 and stopped_confirmed:
                    break
                time.sleep(0.05)
            stop(controller)
            stop(observe)
            (a.output/'termination.txt').write_text(reason+'\n')
            (a.output/'termination_evidence.json').write_text(json.dumps(
                dict(reason=reason, intervention=intervention, stopped_confirmed=stopped_confirmed,
                     command_gate_metrics=dict(command_gate.metrics), contact=state['contact_evidence'],
                     planner_failure=state['planner_failure']), indent=2))
            brake_timer.shutdown()
            command_subscriber.unregister()
            command_publisher.unregister()
            print('DRIVE ENDED', reason, 'speed', state['speed'], flush=True)
            if not stopped_confirmed:
                raise RuntimeError('fresh stopped ego not confirmed; refuse next trial')


if __name__ == '__main__':
    main()
