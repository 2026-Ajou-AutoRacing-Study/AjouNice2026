#!/usr/bin/env python3
"""Bounded independent raw scenes with GT control; production remains unchanged.

The child overlays launch arguments in its own process only. Source validator,
sensor RPM, host timestamp contract and production defaults are not edited.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import yaml

from run_lidar_packet_probe_auto import ROOT, VALIDATOR, prepare, write_json, NETWORK
from run_perception_candidate_probe import POLICIES
import record_lidar_packet_probe as probe


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_cases(output, scene_manifest, policies, missions):
    if output.exists():
        raise FileExistsError('Fresh experiment directory required')
    if not policies or any(policy not in ('fixed51', 'fixed50') for policy in policies):
        raise ValueError('Only explicitly bounded 51/50 grouping comparisons are allowed')
    if len(set(policies)) != len(policies):
        raise ValueError('Duplicate policies are not a separate independent trial')
    if scene_manifest:
        document = json.loads(Path(scene_manifest).read_text())
        scenes = document['scenes']
    else:
        scenes = [dict(name=mission, mission=mission,
                       path=str(ROOT/'configs/scenario/perception_benchmark'/filename))
                  for mission, filename in (('highway', 'bench_highway_merge_raw.json'),
                                            ('roundabout', 'bench_roundabout_raw.json'))]
    scenes = [row for row in scenes if row['mission'] in missions]
    if not scenes or len(scenes) > 4:
        raise ValueError('Require one to four bounded scenes')
    for row in scenes:
        path = Path(row['path']).resolve()
        if row['mission'] not in ('highway', 'roundabout') or not path.is_file():
            raise ValueError('Invalid supported mission scene')
        if row.get('sha256') and digest(path) != row['sha256']:
            raise ValueError('Prepared scene hash mismatch')
        if not row['name'].isascii() or not row['name'].replace('_', '').isalnum():
            raise ValueError('Safe unique scene name required')
    if len({row['name'] for row in scenes}) != len(scenes):
        raise ValueError('Unique scene names required')
    output.mkdir(parents=True)
    cases = []
    for row in scenes:
        for policy in policies:
            folder = output/(row['name']+'_'+policy)
            paths = prepare(folder)
            selected = Path(paths[row['mission']])
            cfg = yaml.safe_load(selected.read_text())
            source = Path(row['path']).resolve()
            cfg['campaign_name'] = 'independent_'+row['name']+'_'+policy
            cfg['scenarios'] = dict(directory=str(source.parent), glob='*.json',
                                    include=[source.name], repeat=1)
            # No video, inference GPU or upload competes with sensor collection.
            cfg['recording']['analysis_mode'] = False
            selected.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
            cases.append(dict(scene=row['name'], mission=row['mission'], policy=policy,
                              config=str(selected.resolve()), source=str(source), source_sha256=digest(source)))
    runner = VALIDATOR/'src/planning_scenario_validator/runner.py'
    plan = dict(cases=cases, scene_manifest_sha256=digest(scene_manifest) if scene_manifest else None,
                validator_source_sha256=digest(runner), production_changed=False,
                scope='GT-controlled raw collection, not learned closed-loop or target LAN latency certification')
    write_json(output/'plan.json', plan)
    return plan


def audit_case(bag, policy):
    import rosbag
    with rosbag.Bag(str(bag)) as stream:
        counts = {topic: info.message_count for topic, info in stream.get_type_and_topic_info().topics.items()}
        packets = probe.packet_summary(message for _, message, _ in stream.read_messages(
            topics=['/sensor/lidar1/packets']))
        duration = stream.get_end_time()-stream.get_start_time()
    expected = 51 if policy == 'fixed51' else 50
    missing = probe.missing_inputs(counts)
    valid = (not missing and packets['packet_counts'] == [expected] and packets['packet_bytes'] == [1206]
             and packets['header_minus_first_packet_sec_max_abs'] is not None
             and packets['header_minus_first_packet_sec_max_abs'] < 1e-6
             and packets['packet_span_sec_min'] >= 0)
    return dict(pass_=valid, missing_inputs=missing, packets=packets, counts=counts,
                duration_sec=duration, policy=policy,
                scope='Input/first-packet contract; not detection accuracy or frame-loss certification')


def restore_final_park(output):
    """Initialize ROS before subscribing and positively restore P/51, no reload."""
    import math
    import time
    import rospy
    from morai_msgs.msg import EgoVehicleStatus
    from std_msgs.msg import Int8
    from velodyne_msgs.msg import VelodyneScan
    from planning_scenario_validator.runtime import Processes, BrakeHold, check_no_conflicting_publishers
    check_no_conflicting_publishers()
    if not rospy.core.is_initialized():
        rospy.init_node('independent_final_park_restore', anonymous=True, disable_signals=True)
    folder = output/'final_restore'
    processes = Processes(folder, os.environ.copy())
    hold = BrakeHold()
    try:
        processes.start('udp', ['roslaunch', 'sensor_interface', 'morai_udp_interface.launch',
            'sensor_config:='+str(ROOT/'configs/sensors/Sensor_competition_local.json'),
            'network_config:='+str(NETWORK), 'enable_control:=false', 'lidar_npackets:=51'])
        hold.start()
        processes.start('park', ['rosrun', 'sensor_interface', 'morai_ctrl_cmd_udp_sender',
            '_network_config:='+str(NETWORK), '_gear:=1', '_ctrl_mode:=2', '__name:=independent_final_park'])
        deadline = time.monotonic()+15.
        stable_since = None
        while time.monotonic() < deadline:
            ego = rospy.wait_for_message('/Ego_topic', EgoVehicleStatus, timeout=3.)
            gear = rospy.wait_for_message('/vehicle/status/gear', Int8, timeout=3.)
            scan = rospy.wait_for_message('/sensor/lidar1/packets', VelodyneScan, timeout=3.)
            speed = math.hypot(ego.velocity.x, ego.velocity.y)
            cut = rospy.get_param('/lidar/lidar_driver/cut_angle')
            if gear.data == 1 and speed < .1 and len(scan.packets) == 51 and cut < 0:
                stable_since = stable_since or time.monotonic()
                if time.monotonic()-stable_since >= .5:
                    write_json(folder/'result.json', dict(verified=True, park_verified=True, gear=gear.data,
                        speed_mps=speed, packets=len(scan.packets), cut_angle=cut,
                        position=[ego.position.x, ego.position.y]))
                    return
            else:
                stable_since = None
        raise RuntimeError('Actual Park/51 restore not confirmed; no scene reload')
    finally:
        processes.close()
        hold.stop()
        # This experiment owns the temporary npackets override. Removing it
        # after all owned nodes exit restores normal automatic 51 calculation.
        if rospy.has_param('/lidar/lidar_driver/npackets'):
            rospy.delete_param('/lidar/lidar_driver/npackets')


def run_cases(output, plan):
    from planning_scenario_validator.config import load_campaign
    # Fail before any scene load if one planned config is invalid.
    for row in plan['cases']:
        cfg = load_campaign(row['config'])
        if '/sensor/lidar1/packets' not in cfg['recording']['topics']:
            raise ValueError('Missing raw packets topic')
    completed = []
    for row in plan['cases']:
        if digest(row['source']) != row['source_sha256']:
            raise RuntimeError('Scene changed after preflight')
        command = [sys.executable, str(Path(__file__).resolve()), '--child-config', row['config'],
                   '--child-policy', row['policy']]
        with (output/(row['scene']+'_'+row['policy']+'.log')).open('w') as log:
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = child.wait(timeout=300)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                probe.stop_recorder(child)
                raise RuntimeError('Bounded child interrupted; raw evidence preserved')
        if code:
            raise RuntimeError('Child failed; evidence preserved: '+row['config'])
        config = Path(row['config'])
        states = list((config.parent/'validator').glob('independent_*/campaign_state.json'))
        if len(states) != 1:
            raise RuntimeError('Ambiguous campaign results')
        state = json.loads(states[0].read_text())
        if not state.get('results') or any(value['status'] != 'PASS' for value in state['results'].values()):
            raise RuntimeError('Motion/contact validation failed; no automatic retry or promotion')
        bags = list(states[0].parent.rglob('*.bag'))
        if len(bags) != 1:
            raise RuntimeError('Expected one finalized raw bag')
        audit = audit_case(bags[0], row['policy'])
        write_json(states[0].parent/'independent_input_audit.json', audit)
        if not audit['pass_']:
            raise RuntimeError('Raw input contract failed')
        completed.append(dict(**row, folder=str(bags[0].parent.resolve()),
                              bag=str(bags[0].resolve()), bag_sha256=digest(bags[0]), audit=audit))
        write_json(output/'progress.json', dict(completed=completed))
        print(json.dumps(dict(event='case_completed', scene=row['scene'], policy=row['policy'],
                              bag=completed[-1]['bag']), ensure_ascii=False), flush=True)
    runner = VALIDATOR/'src/planning_scenario_validator/runner.py'
    if digest(runner) != plan['validator_source_sha256']:
        raise RuntimeError('Validator source changed during experiment; source versions must be reconciled')
    restore_final_park(output)
    write_json(output/'complete.json', dict(cases=len(completed), production_changed=False,
                                          validator_source_unchanged=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--scene-manifest', type=Path)
    parser.add_argument('--policy', action='append', choices=('fixed51', 'fixed50'))
    parser.add_argument('--mission', action='append', choices=('highway', 'roundabout'))
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--finalize-existing', action='store_true',
                        help='Restore Park/51 after every raw case passed, without repeating any drive')
    parser.add_argument('--child-config', type=Path)
    parser.add_argument('--child-policy', choices=('fixed51', 'fixed50'))
    args = parser.parse_args()
    if args.child_config:
        if not args.child_policy:
            parser.error('Child requires explicit grouping policy')
        from planning_scenario_validator import runner
        original = runner._launch
        original_load = runner._load_scene
        parking = [None]
        parking_log = []

        def stop_parking():
            if parking[0] is not None:
                probe.stop_recorder(parking[0])
                parking[0] = None
            if parking_log:
                parking_log.pop().close()

        def parked_load(cfg, installed, monitor, ego_xy):
            # A successful position load alone did not guarantee stationary
            # ExternalCtrl: the failed first trial moved during route startup.
            # Park/brake owns the sole control transport until route readiness.
            import math
            import rospy
            from std_msgs.msg import Int8
            from run_lidar_packet_probe_auto import NETWORK
            stop_parking()
            handle = (args.child_config.parent/'park_prepare.log').open('ab')
            parking_log.append(handle)
            parking[0] = subprocess.Popen([
                'rosrun', 'sensor_interface', 'morai_ctrl_cmd_udp_sender',
                '_network_config:='+str(NETWORK), '_gear:=1', '_ctrl_mode:=2',
                '__name:=independent_prepare_park'], stdout=handle, stderr=subprocess.STDOUT,
                start_new_session=True)
            original_load(cfg, installed, monitor, ego_xy)
            def stationary():
                if parking[0].poll() is not None:
                    raise RuntimeError('Preparation Park sender failed')
                gear = rospy.wait_for_message('/vehicle/status/gear', Int8, timeout=2)
                return (gear.data == 1 and monitor.ego is not None
                        and math.hypot(monitor.ego.velocity.x, monitor.ego.velocity.y) < .1
                        and math.hypot(monitor.ego.position.x-ego_xy[0], monitor.ego.position.y-ego_xy[1]) < 2.)
            monitor.wait(stationary, 12, 'actual P and stationary loaded Ego')
            monitor.event('RESEARCH_PREPARATION_PARK_VERIFIED')

        def research_launch(package, filename, **kwargs):
            if package == 'bringup' and filename == 'maptf_morai.launch':
                filename = 'localization_morai_gt.launch'
                kwargs['use_heading_correction'] = True
            command = original(package, filename, **kwargs)
            if package == 'sensor_interface' and filename == 'morai_udp_interface.launch' and kwargs.get('enable_control') is True:
                stop_parking()
            if package == 'sensor_interface' and filename == 'morai_udp_interface.launch' and kwargs.get('enable_lidar'):
                command += POLICIES[args.child_policy]
            return command

        runner._launch = research_launch
        runner._load_scene = parked_load
        try:
            return runner.main([str(args.child_config)])
        finally:
            stop_parking()
    if not args.output:
        parser.error('Parent requires fresh --output')
    if args.finalize_existing:
        if args.run:
            parser.error('Finalization cannot start a new campaign')
        plan = json.loads((args.output/'plan.json').read_text())
        progress = json.loads((args.output/'progress.json').read_text())
        if len(progress['completed']) != len(plan['cases']) or any(not r['audit']['pass_'] for r in progress['completed']):
            raise RuntimeError('Finalization requires every raw case to have passed')
        for row in progress['completed']:
            if digest(row['bag']) != row['bag_sha256']:
                raise RuntimeError('Raw evidence changed before finalization')
        restore_final_park(args.output)
        write_json(args.output/'complete.json', dict(cases=len(progress['completed']), production_changed=False,
            finalization_only=True, runtime_source_sha256=digest(__file__), raw_drives_not_repeated=True))
        return 0
    plan = prepare_cases(args.output, args.scene_manifest,
                         args.policy or ['fixed51', 'fixed50'], args.mission or ['highway', 'roundabout'])
    if args.run:
        try:
            run_cases(args.output, plan)
        except Exception as error:
            # The child preserves brake-through-cleanup. Positively restore
            # Park/51 after failure as well, without retrying the failed drive.
            write_json(args.output/'failure.json', dict(error_type=type(error).__name__, error=str(error)))
            try:
                restore_final_park(args.output)
            except Exception as cleanup_error:
                write_json(args.output/'cleanup_failure.json', dict(error_type=type(cleanup_error).__name__,
                    error=str(cleanup_error), original_error=str(error)))
            raise
    return 0


if __name__ == '__main__':
    sys.exit(main())
