#!/usr/bin/env python3
"""Explicit non-resumable development A/B through the REAL validator runner.

Keeps the validator's ownership, preparation, real planner/controller and final
stop/P cleanup. Only explicit, recorded launch arguments and an optional signal
fault proxy differ. Native lamp state is NEVER converted to planner input.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
SOURCE_TOPIC = '/evaluation/traffic_light_policy/source_signals'
FAULT_TOPIC = '/evaluation/traffic_light_policy/fault'
CANONICAL = '/perception/traffic_light_recognition/traffic_signals'


def launch_arguments(package, filename, arguments, guarded, route_progress, proxy,
                     temporal_direct=False, signal_timeout_sec=1.0):
    result = dict(arguments)
    if package == 'ajounice_planning_integration' and filename == 'planning.launch':
        result.update(enable_signal_compliance_guard=guarded,
                      signal_compliance_front_axle_offset_m=3.0,
                      traffic_light_state_timeout_sec=signal_timeout_sec)
    if package == 'traffic_light_detection' and filename in (
            'traffic_light_node.launch', 'traffic_light_v5_candidate.launch'):
        result['route_progress_guard'] = route_progress
        result['planner_compliance_observations'] = temporal_direct
        if proxy:
            result['output_topic'] = SOURCE_TOPIC
    return result


def fault_copy(message, mode, group_id):
    """Only drop or alter identity; never fabricate a color or refresh a stamp."""
    if mode == 'drop':
        return None
    if mode != 'wrong_id':
        return message
    output = copy.deepcopy(message)
    for group in output.traffic_light_groups:
        if group.traffic_light_group_id == group_id:
            group.traffic_light_group_id = 0
    return output


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require_trial_contract(cfg):
    safety = cfg.get('localization_safety', {})
    if (cfg['stack'].get('pose_source') != 'localization' or
            not safety.get('enabled') or not safety.get('park_after_run') or
            len(cfg.get('cases', [])) != 1):
        raise ValueError('Require one real-localization case with verified stop/P safety enabled')


def signal_topic_conflicts(publishers):
    return {topic: publishers[topic] for topic in (CANONICAL, SOURCE_TOPIC, FAULT_TOPIC)
            if publishers.get(topic)}


def active_fault_mode(state, now):
    if state.get('received') and not state.get('terminal') and now-state['receipt_monotonic'] > .5:
        raise RuntimeError('Fault-control lease expired; experiment is not qualified')
    return state['mode']


def run_owned_trial(prepare_helpers, run_campaign, close_helpers):
    """Cleanup remains inside the owner's graph lock even if startup fails."""
    try:
        prepare_helpers()
        return run_campaign()
    finally:
        close_helpers()


def main():
    from control_traffic_light_policy_trial import parse_yellow_durations
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--mode', choices=('baseline', 'guarded'), required=True)
    parser.add_argument('--route-progress', choices=('true', 'false'), default='true')
    parser.add_argument('--experiment', choices=('natural', 'stopped-yellow', 'missing-signal', 'green-loss'), default='natural')
    parser.add_argument('--temporal-direct', choices=('true','false'), default='false')
    parser.add_argument('--signal-timeout-sec', type=float, default=1.0)
    parser.add_argument('--group-id', type=int, choices=(201351, 201365, 201290, 201326), default=201326)
    parser.add_argument('--yellow-durations-sec', type=parse_yellow_durations,
                        help='Explicit stopped-yellow stress durations, for example 2,3; not an operating-time assumption')
    parser.add_argument('--native-python', type=Path,
                        default=Path.home()/'miniconda3/envs/traffic_light_gpu/bin/python')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.yellow_durations_sec is not None and args.experiment != 'stopped-yellow':
        parser.error('Explicit yellow durations apply only to the stopped-yellow experiment')
    if not .2 <= args.signal_timeout_sec <= 2.0:
        parser.error('Observation freshness timeout must be within 0.2..2 seconds')
    sys.path.insert(0, str(ROOT/'base_ws/src/planning_addons/planning_scenario_validator/src'))
    from planning_scenario_validator import runner
    from planning_scenario_validator.traffic_light import profile_name
    from planning_scenario_validator.config import load_campaign
    from observe_traffic_light_compliance import load_geometry
    cfg = load_campaign(args.config)
    require_trial_contract(cfg)
    if not cfg['stack'].get('traffic_light') or profile_name(cfg['stack']) != 'v5_candidate':
        parser.error('Require an explicit real v5 traffic-light trial')
    args.output = args.output.resolve()
    args.output.relative_to(ROOT/'artifacts/evaluation')
    if args.output.exists():
        parser.error('Use a fresh evidence directory; this wrapper NEVER resumes')
    if not args.native_python.is_file() or not args.geometry.is_file():
        parser.error('Existing native Python and explicit geometry are required')
    load_geometry(args.geometry)
    guarded, progress = args.mode == 'guarded', args.route_progress == 'true'
    proxy = args.experiment != 'natural'
    receipt = dict(mode=args.mode, route_progress=progress, experiment=args.experiment,
        planner_compliance_observations=args.temporal_direct=='true',
        signal_observation_timeout_sec=args.signal_timeout_sec,
        yellow_durations_sec=(list(args.yellow_durations_sec or (1., 3., 5.))
                              if args.experiment == 'stopped-yellow' else None),
        group_id=args.group_id, source_config=str(args.config.resolve()),
        config_sha256=digest(args.config), geometry_sha256=digest(args.geometry),
        wrapper_sha256=digest(__file__), native_python=str(args.native_python),
        front_axle_offset_m=3.0,
        frame_provenance='configs/planning/vehicle_info.yaml and vehicle/README.md rear-axle base_link',
        resume_allowed=False, native_GT_is_planner_input=False,
        planner='real ROS1 planner/controller through unmodified validator runner')
    if args.check:
        print(json.dumps(receipt, indent=2))
        return runner.main([str(args.config), '--check'])
    # Refuse another person's stack/proxy BEFORE starting either native helper.
    import rosgraph
    from planning_scenario_validator.runtime import check_no_conflicting_publishers
    check_no_conflicting_publishers()
    conflicts = signal_topic_conflicts(dict(rosgraph.Master('/signal_policy_trial_preflight').getSystemState()[0]))
    if conflicts:
        raise RuntimeError('Existing signal/fault publisher; nothing started: '+repr(conflicts))
    args.output.mkdir(parents=True)
    (args.output/'selection.json').write_text(json.dumps(receipt, indent=2)+'\n')
    lock, done, fatal = threading.Lock(), threading.Event(), threading.Event()
    phase_teardown = threading.Event()
    children, handles, subscriptions = [], [], []
    state = dict(mode='none', group_id=args.group_id, receipt_monotonic=0., received=False, terminal=False)

    def event(value):
        value = dict(value, monotonic=time.monotonic(), wall_time=time.time())
        with lock, (args.output/'wrapper_events.jsonl').open('a') as stream:
            stream.write(json.dumps(value, allow_nan=False)+'\n')

    def start(name, command):
        handle = (args.output/(name+'.log')).open('x')
        handles.append(handle)
        process = subprocess.Popen(command, cwd=ROOT, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        children.append((name, process))
        event(dict(event='child_started', name=name, command=command, pid=process.pid))
        return process

    original_launch = runner._launch
    original_stop = runner._stop_and_verify
    original_monitor = runner.LiveMonitor
    original_campaign = runner._run_campaign
    proxy_started = False
    publisher = None
    def selected_launch(package, filename, **kwargs):
        nonlocal proxy_started, publisher
        selected = launch_arguments(package, filename, kwargs, guarded, progress, proxy,
            temporal_direct=args.temporal_direct=='true', signal_timeout_sec=args.signal_timeout_sec)
        if proxy and package == 'traffic_light_detection' and not proxy_started:
            import rospy
            from autoware_perception_msgs.msg import TrafficLightGroupArray
            from std_msgs.msg import String
            publisher = rospy.Publisher(CANONICAL, TrafficLightGroupArray, queue_size=1)
            def control(message):
                try:
                    body = json.loads(message.data)
                    if (not isinstance(body, dict) or body.get('mode') not in ('none','drop','wrong_id')
                            or body.get('group_id') != args.group_id):
                        raise ValueError('Invalid fault control')
                    with lock:
                        state.update(mode=body['mode'], group_id=body['group_id'], receipt_monotonic=time.monotonic(),
                            received=True, terminal=body['mode']=='none' and body.get('stage') in ('finished','aborted'))
                except Exception as error:
                    fatal.set()
                    event(dict(event='invalid_fault_control', error_type=type(error).__name__))
            def forward(message):
                try:
                    with lock:
                        mode = 'none' if phase_teardown.is_set() else active_fault_mode(state, time.monotonic())
                    output = fault_copy(message, mode, args.group_id)
                    event(dict(event='signal_proxy', mode=mode,
                        source_stamp_ns=int(message.stamp.sec)*10**9+int(message.stamp.nanosec),
                        input_ids=[int(group.traffic_light_group_id) for group in message.traffic_light_groups],
                        output_ids=None if output is None else [int(group.traffic_light_group_id) for group in output.traffic_light_groups]))
                    if output is not None:
                        publisher.publish(output)
                except Exception as error:
                    fatal.set()
                    event(dict(event='signal_proxy_error', error_type=type(error).__name__))
            subscriptions.extend((rospy.Subscriber(FAULT_TOPIC,String,control,queue_size=1),
                                  rospy.Subscriber(SOURCE_TOPIC,TrafficLightGroupArray,forward,queue_size=1)))
            proxy_started = True
        command = original_launch(package, filename, **selected)
        event(dict(event='launch', package=package, filename=filename, command=command))
        return command

    def begin_phase_teardown():
        if phase_teardown.is_set():
            return
        phase_teardown.set()
        event(dict(event='validator_safety_teardown_started'))
        for name, process in children:
            if name == 'phase_controller' and process.poll() is None:
                process.terminate()

    class TrialMonitor(original_monitor):
        def event(self, name, **fields):
            # CASE_RESULT precedes controller stop, which may take >0.5s.
            if name == 'CASE_RESULT':
                begin_phase_teardown()
            return super().event(name, **fields)

    def selected_stop(cfg, processes, monitor):
        # A normal goal deliberately stops /ctrl_cmd before stop/P verification.
        # Tell the phase helper that its freshness contract has ended BEFORE
        # teardown, so it cannot interrupt the validator's safety cleanup.
        begin_phase_teardown()
        return original_stop(cfg, processes, monitor)

    old_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt()
    for sig in old_handlers:
        signal.signal(sig, interrupted)
    helpers_closed = False
    def close_helpers():
        nonlocal helpers_closed
        if helpers_closed:
            return
        helpers_closed = True
        done.set()
        begin_phase_teardown()
        for subscription in subscriptions:
            subscription.unregister()
        if publisher is not None:
            publisher.unregister()
        for name, process in reversed(children):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    event(dict(event='owned_child_timeout', name=name))
                    process.kill(); process.wait(timeout=3)
            event(dict(event='child_finished', name=name, returncode=process.returncode))
        for handle in handles:
            handle.close()

    def selected_campaign(*campaign_args, **campaign_kwargs):
        # runner.run holds its global ROS-graph flock around this callback.
        # Native helpers cannot run before lock acquisition or outlive release.
        check_no_conflicting_publishers()
        conflicts = signal_topic_conflicts(dict(rosgraph.Master('/signal_policy_trial_locked_preflight').getSystemState()[0]))
        if conflicts:
            raise RuntimeError('Signal ownership changed before graph lock: '+repr(conflicts))
        event(dict(event='global_validator_graph_lock_owned'))
        def prepare_helpers():
            duration = min(1800, max(case.timeout_sec for case in cfg['cases']) + 300)
            start('observer', [str(args.native_python), str(ROOT/'scripts/evaluation/observe_traffic_light_compliance.py'),
                '--output', str(args.output/'observer'), '--geometry', str(args.geometry), '--duration-sec',str(duration),
                '--crossing-jpegs'])
            if proxy:
                phase_script = ('control_traffic_light_green_loss_trial.py' if args.experiment=='green-loss'
                                else 'control_traffic_light_policy_trial.py')
                phase_command = [str(args.native_python), str(ROOT/'scripts/evaluation'/phase_script),
                    '--output',str(args.output/'phase_controller'),'--geometry',str(args.geometry),
                    '--group-id',str(args.group_id),'--experiment',args.experiment,'--max-seconds','180']
                if args.yellow_durations_sec is not None:
                    phase_command.extend(('--yellow-durations-sec',
                        ','.join(format(value, 'g') for value in args.yellow_durations_sec)))
                start('phase_controller', phase_command)
            def watch():
                while not done.wait(.2):
                    try:
                        with lock:
                            if not phase_teardown.is_set():
                                active_fault_mode(state, time.monotonic())
                    except RuntimeError:
                        fatal.set()
                    if fatal.is_set() and not phase_teardown.is_set():
                        event(dict(event='proxy_fatal_abort'))
                        os.kill(os.getpid(), signal.SIGTERM)
                        return
                    for name, process in children:
                        code = process.poll()
                        if (code is not None and (code != 0 or name == 'observer') and
                                not (name == 'phase_controller' and phase_teardown.is_set())):
                            event(dict(event='child_error_abort', name=name, returncode=code))
                            os.kill(os.getpid(), signal.SIGTERM)
                            return
            threading.Thread(target=watch, daemon=True).start()
        return run_owned_trial(prepare_helpers,
            lambda: original_campaign(*campaign_args, **campaign_kwargs), close_helpers)

    try:
        runner._launch = selected_launch
        runner._stop_and_verify = selected_stop
        runner.LiveMonitor = TrialMonitor
        runner._run_campaign = selected_campaign
        code = runner.main([str(args.config), '--resume','false'])
        event(dict(event='validator_returned', returncode=code))
        return code
    finally:
        done.set()
        runner._launch = original_launch
        runner._stop_and_verify = original_stop
        runner.LiveMonitor = original_monitor
        runner._run_campaign = original_campaign
        close_helpers()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
