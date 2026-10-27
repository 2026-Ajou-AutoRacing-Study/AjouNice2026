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
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
SOURCE_TOPIC = '/evaluation/traffic_light_policy/source_signals'
FAULT_TOPIC = '/evaluation/traffic_light_policy/fault'
CANONICAL = '/perception/traffic_light_recognition/traffic_signals'
FIXTURE_TOPIC = '/evaluation/traffic_light_policy/planner_fixture_signals'
FIXTURE_STATUS_TOPIC = '/evaluation/traffic_light_policy/planner_fixture_status'
FIXTURE_MODES = ('stopped-yellow', 'moving-yellow-stoppable', 'moving-yellow-close',
                 'loss-prefront', 'loss-postfront', 'postfront-red')


def launch_arguments(package, filename, arguments, guarded, route_progress, proxy,
                     temporal_direct=False, signal_timeout_sec=1.0, yellow_passthrough=False,
                     visual_continuation_sec=None, perception_odometry_topic=None):
    result = dict(arguments)
    if perception_odometry_topic is not None and (
            not isinstance(perception_odometry_topic, str) or
            not perception_odometry_topic.startswith('/evaluation/traffic_light_policy/') or
            any(ch.isspace() for ch in perception_odometry_topic)):
        raise ValueError('Perception-only stress odometry must use the explicit evaluation namespace')
    if visual_continuation_sec is not None and (
            isinstance(visual_continuation_sec, bool) or
            not isinstance(visual_continuation_sec, (int, float)) or
            not 0 <= visual_continuation_sec <= .3):
        raise ValueError('visual continuation must be finite 0..0.3 seconds')
    if guarded is None:
        if proxy or visual_continuation_sec is not None or perception_odometry_topic is not None:
            raise ValueError('Unmodified production defaults cannot use a signal proxy')
        return result
    if package == 'ajounice_planning_integration' and filename == 'planning.launch':
        result.update(enable_signal_compliance_guard=guarded,
                      enable_yellow_passthrough=yellow_passthrough,
                      # Freeze historical trial baselines across launch promotion.
                      # Explicit temporal_video_launch_arguments overrides this.
                      signal_compliance_unknown_grace_sec=0.0,
                      signal_compliance_front_axle_offset_m=3.0,
                      traffic_light_state_timeout_sec=signal_timeout_sec)
        if not guarded:
            result.update(defer_unseen_until_observed=False,
                          defer_far_unseen_unknown=False,
                          signal_compliance_enable_timed_yellow=False)
    if package == 'traffic_light_detection' and filename in (
            'traffic_light_node.launch', 'traffic_light_v5_candidate.launch'):
        result['route_progress_guard'] = route_progress
        result['planner_compliance_observations'] = temporal_direct
        result['preserve_unreadable_color_evidence'] = False
        if visual_continuation_sec is not None:
            result['visual_continuation_sec'] = visual_continuation_sec
        if perception_odometry_topic is not None:
            result['odometry_topic'] = perception_odometry_topic
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


def map_launch_arguments(package, filename, arguments, map_file, map_sha256):
    """Use one explicit candidate for planning and signal route geometry alike."""
    result = dict(arguments)
    if map_file is None:
        return result
    if package == 'ajounice_planning_integration' and filename == 'planning.launch':
        result.update(map_file=str(map_file), map_sha256=map_sha256)
    if package == 'traffic_light_detection' and filename in (
            'traffic_light_node.launch', 'traffic_light_v5_candidate.launch'):
        result['lanelet2_map_file'] = str(map_file)
    if package == 'perception_object_pipeline' and filename == 'object_pipeline.launch':
        result['map_file'] = str(map_file)
    if package == 'bringup' and filename == 'aw149_planning_rviz.launch':
        result['map_file'] = str(map_file)
    return result


def receipt_yellow_durations(experiment, requested):
    if experiment not in ('stopped-yellow', 'first-observation-yellow', 'moving-native-yellow'):
        return None
    default = (3.,) if experiment == 'first-observation-yellow' else (1., 3., 5.)
    return list(default if requested is None else requested)


def validate_short_cycle_request(experiment, plan, mode, temporal_direct):
    """Renderer stress is explicit and never substitutes native GT for vision."""
    enabled = experiment == 'short-native-cycles'
    if enabled != (plan is not None):
        raise ValueError('short-native-cycles requires --short-cycle-plan, and only that experiment accepts it')
    if enabled and (mode != 'guarded' or temporal_direct != 'true'):
        raise ValueError('Short native cycles require guarded direct-observation mode')
    if enabled:
        body = json.loads(Path(plan).read_text())
        if not isinstance(body, dict):
            raise ValueError('Short-cycle plan must be a JSON object')
    return enabled


class CrossingVideoDeadline:
    """Read only newly written observer rows; never stop driving on a video cut."""
    def __init__(self, path, group_id, tail_sec, direction='negative_to_positive'):
        if (group_id != 201326 or isinstance(tail_sec, bool)
                or not isinstance(tail_sec, (int, float)) or not math.isfinite(tail_sec)
                or not 0 < tail_sec <= 30
                or direction not in ('negative_to_positive', 'positive_to_negative')):
            raise ValueError('Video cut requires D group201326, a directed crossing and a finite0..30s positive tail')
        self.path, self.group_id, self.tail_sec = Path(path), group_id, tail_sec
        self.direction = direction
        self.offset, self.pending, self.deadline, self.trigger = 0, b'', None, None

    def poll(self, now):
        if self.deadline is None and self.path.is_file():
            with self.path.open('rb') as stream:
                stream.seek(self.offset)
                data = stream.read(4 * 1024 * 1024)
                self.offset = stream.tell()
            rows = (self.pending + data).split(b'\n')
            self.pending = rows.pop()
            if len(self.pending) > 4 * 1024 * 1024:
                raise ValueError('Observer JSONL row exceeds the bounded video-cut reader')
            for row in rows:
                try:
                    body = json.loads(row)
                except (ValueError, UnicodeDecodeError):
                    continue
                if (isinstance(body, dict)
                        and body.get('event') == 'front_point_segment_crossing_pending_native_bracket'
                        and body.get('group_id') == self.group_id
                        and body.get('pose_time_verified') is True
                        and body.get('direction') == self.direction):
                    self.trigger = body
                    # Receipt now is conservative if polling was delayed.
                    self.deadline = now + self.tail_sec
                    break
        return self.deadline is not None and now >= self.deadline


def temporal_video_launch_arguments(package, filename, arguments, required_green=None,
                                    stamped_vote=None, publish_debug=False, *,
                                    stamped_vote_max_age_sec=None,
                                    stamped_vote_startup_passthrough=None,
                                    unknown_grace_sec=None):
    """Explicit trial-only changes; never select legacy corrections to get a vote."""
    result = dict(arguments)
    if required_green is not None and (type(required_green) is not int or not 1 <= required_green <= 3):
        raise ValueError('required_green must be 1..3')
    if unknown_grace_sec is not None and (
            isinstance(unknown_grace_sec, bool) or not isinstance(unknown_grace_sec, (int, float))
            or not math.isfinite(unknown_grace_sec) or not 0 <= unknown_grace_sec <= .3):
        raise ValueError('unknown grace must be finite 0..0.3 seconds')
    if unknown_grace_sec and stamped_vote not in (None, 0):
        raise ValueError('UNKNOWN grace trial cannot also enable a phase-voting candidate')
    if stamped_vote is not None and (type(stamped_vote) is not int or stamped_vote not in (0, 3, 7)):
        raise ValueError('stamped_vote must be 0, 3 or 7')
    if stamped_vote_max_age_sec is not None:
        cap = .9 if stamped_vote == 7 else .5
        if (stamped_vote not in (3, 7) or isinstance(stamped_vote_max_age_sec, bool)
                or not isinstance(stamped_vote_max_age_sec, (int, float))
                or not math.isfinite(stamped_vote_max_age_sec)
                or not 0 < stamped_vote_max_age_sec <= cap):
            raise ValueError('explicit vote age requires enabled window and its bounded age cap')
    if stamped_vote_startup_passthrough is not None and (
            stamped_vote not in (3, 7) or type(stamped_vote_startup_passthrough) is not bool):
        raise ValueError('startup passthrough requires enabled window and an explicit bool')
    if package == 'ajounice_planning_integration' and filename == 'planning.launch':
        if required_green is not None:
            result['signal_compliance_required_green_observations'] = required_green
        if unknown_grace_sec is not None:
            result['signal_compliance_unknown_grace_sec'] = unknown_grace_sec
    if package == 'traffic_light_detection' and filename in (
            'traffic_light_node.launch', 'traffic_light_v5_candidate.launch'):
        if stamped_vote is not None:
            result['stamped_vote_window_size'] = stamped_vote
        if stamped_vote_max_age_sec is not None:
            result['stamped_vote_max_age_sec'] = stamped_vote_max_age_sec
        if stamped_vote_startup_passthrough is not None:
            result['stamped_vote_startup_passthrough'] = stamped_vote_startup_passthrough
        if publish_debug:
            result['publish_debug_image'] = True
        if unknown_grace_sec is not None:
            result['preserve_unreadable_color_evidence'] = unknown_grace_sec > 0
            if unknown_grace_sec > 0:
                # Do not inherit a phase-vote/legacy handoff from an old YAML.
                # This explicit trial is the direct-observation alternative.
                result['stamped_vote_window_size'] = 0
                result['planner_compliance_observations'] = True
    return result


def simulator_capture_geometry(window_id, display):
    """Record only the explicitly chosen Simulator window, not a private desktop."""
    if not re.fullmatch(r'0x[0-9a-fA-F]+', window_id):
        raise ValueError('Explicit hexadecimal Simulator window ID required')
    env = dict(os.environ, DISPLAY=display)
    identity = subprocess.check_output(['xprop','-id',window_id,'WM_CLASS'],env=env,text=True,timeout=5)
    if 'Simulator.x86_64' not in identity:
        raise ValueError('Selected window is not the MORAI Simulator')
    info = subprocess.check_output(['xwininfo','-id',window_id],env=env,text=True,timeout=5)
    values = []
    for key in ('Absolute upper-left X','Absolute upper-left Y','Width','Height'):
        found = re.search(re.escape(key)+r':\s*(-?\d+)', info)
        if found is None:
            raise ValueError('Missing X11 window geometry')
        values.append(int(found[1]))
    x,y,w,h = values
    if min(x,y)<0 or w<320 or h<240 or 'IsViewable' not in info:
        raise ValueError('Simulator window is not visibly captureable')
    return dict(window_id=window_id,display=display,x=x,y=y,width=w//2*2,height=h//2*2)


def activate_simulator_for_capture(window_id, display):
    """Explicit video operation: prevent an editor covering the capture region."""
    code = '''import gi,sys,time
gi.require_version('Wnck','3.0');gi.require_version('Gdk','3.0');gi.require_version('GdkX11','3.0')
from gi.repository import Wnck,Gdk,GdkX11,GLib
s=Wnck.Screen.get_default();s.force_update();wid=int(sys.argv[1],16)
w=next(w for w in s.get_windows() if w.get_xid()==wid)
w.activate(GdkX11.x11_get_server_time(Gdk.get_default_root_window()))
c=GLib.MainContext.default()
for _ in range(10):
 while c.pending():c.iteration(False)
 time.sleep(.05)
s.force_update()
if s.get_active_window() is None or s.get_active_window().get_xid()!=wid:
 raise RuntimeError('Simulator could not be raised for screen recording')
'''
    subprocess.run(['/usr/bin/python3','-c',code,window_id],check=True,
                   env=dict(os.environ,DISPLAY=display),timeout=5,capture_output=True)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def owned_child_exit_grace_sec(name):
    # Recorder EOF finalization allows 20s plus a bounded encoder fallback.
    return 30 if name == 'front_video' else 8


def validate_video_request(enabled, front_only, label, window_id):
    if front_only and not enabled:
        raise ValueError('Front-only recording requires --record-video')
    if enabled and (not label or not front_only and not window_id):
        raise ValueError('Video requires a label and, unless front-only, an explicit Simulator window ID')


def require_trial_contract(cfg):
    safety = cfg.get('localization_safety', {})
    if (cfg['stack'].get('pose_source') != 'localization' or
            not safety.get('enabled') or not safety.get('park_after_run') or
            len(cfg.get('cases', [])) != 1):
        raise ValueError('Require one real-localization case with verified stop/P safety enabled')


def signal_topic_conflicts(publishers):
    return {topic: publishers[topic] for topic in (
            CANONICAL, SOURCE_TOPIC, FAULT_TOPIC, FIXTURE_TOPIC, FIXTURE_STATUS_TOPIC)
            if publishers.get(topic)}


def active_fault_mode(state, now):
    if state.get('received') and not state.get('terminal') and now-state['receipt_monotonic'] > .5:
        raise RuntimeError('Fault-control lease expired; experiment is not qualified')
    return state['mode']


def fixture_relay_mode(state, now):
    """An explicit, live completed status is the ONLY handoff to real vision."""
    if not state.get('received'):
        if now-state['started_monotonic'] > 15.:
            raise RuntimeError('Planner fixture did not publish its startup heartbeat')
        return 'fixture'
    if not 0 <= now-state['receipt_monotonic'] <= .5:
        raise RuntimeError('Planner fixture heartbeat expired or clock reversed')
    if state.get('error'):
        raise RuntimeError('Planner fixture error/not-applicable; never fall back to vision')
    return 'real' if state.get('handoff_real') else 'fixture'


def validated_fixture_status(body, group_id, mode):
    if (not isinstance(body, dict) or body.get('group_id') != group_id or
            body.get('mode') != mode or body.get('planner_only') is not True):
        raise ValueError('Incorrect planner-only fixture identity/scope')
    handoff = body.get('handoff_real', False)
    if type(handoff) is not bool:
        raise ValueError('Fixture handoff must be an explicit bool')
    if handoff and (body.get('stage') != 'fixture_complete' or
                    body.get('motion_stage_captured') is not True):
        raise ValueError('Fixture handoff lacks an observed, completed motion stage')
    return dict(handoff_real=handoff,
                error=bool(body.get('error')) or body.get('stage') in ('error', 'not_applicable'))


def with_compliance_debug(command, package, filename, config):
    if config is not None and package == 'ajounice_planning_integration' and filename == 'planning.launch':
        if not isinstance(command, (list, tuple)):
            raise TypeError('Validator launch commands must remain argv lists, not shell strings')
        return list(command) + ['planning_rosconsole_config:='+str(config)]
    return command


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
    parser.add_argument('--map-file', type=Path,
                        help='Explicit map candidate shared by planning and signal route geometry')
    parser.add_argument('--preparation-geometry', type=Path,
                        help='Optional virtual stop-target geometry for stationary control gate; observer still uses --geometry')
    parser.add_argument('--mode', choices=('baseline', 'guarded', 'production'), required=True,
                        help='production leaves policy launch defaults untouched; natural signals only')
    parser.add_argument('--route-progress', choices=('true', 'false'), default='true')
    parser.add_argument('--experiment', choices=('natural', 'stopped-yellow', 'first-observation-yellow', 'missing-signal', 'green-loss', 'moving-native-yellow', 'short-native-cycles'), default='natural')
    parser.add_argument('--short-cycle-plan', type=Path,
                        help='Explicit native-rendered phase plan; development stress, never natural signals')
    parser.add_argument('--temporal-direct', choices=('true','false'), default='false')
    parser.add_argument('--visual-continuation-sec', type=float,
                        help='Explicit bounded visual-identity candidate (0..0.3s), not production mode')
    parser.add_argument('--observation-start-depth-m', type=float,
                        help='Trial only: use lamps after first current map-matched housing at this camera depth')
    parser.add_argument('--perception-odometry-topic',
                        help='Explicit evaluation-only covariance stress relay; never changes planner/TF input')
    parser.add_argument('--signal-timeout-sec', type=float, default=1.0)
    parser.add_argument('--yellow-passthrough', choices=('true', 'false'), default='false',
                        help='Explicit pre-line yellow dilemma override; default OFF, production keeps launch defaults')
    parser.add_argument('--group-id', type=int, choices=(201351, 201365, 201290, 201326), default=201326)
    parser.add_argument('--yellow-durations-sec', type=parse_yellow_durations,
                        help='Explicit stopped-yellow stress durations, for example 2,3; not an operating-time assumption')
    parser.add_argument('--trigger-front-distance-m', type=float, default=8.,
                        help='Real moving-native-yellow trigger distance from front axle to finite stopline')
    parser.add_argument('--trigger-distance-basis', choices=('finite_front', 'planner_target'), default='finite_front')
    parser.add_argument('--moving-red-hold-sec', type=float, default=3.)
    parser.add_argument('--evaluation-speed-cap-mps', type=float,
                        help='Owned development-only internal speed ceiling, not a vehicle command')
    parser.add_argument('--planner-fixture', choices=FIXTURE_MODES,
                        help='Explicit synthetic planner-only input; no native SET or perception qualification')
    parser.add_argument('--fixture-probe-seconds', type=float, choices=(2., 3.), default=2.)
    parser.add_argument('--compliance-debug', action='store_true',
                        help='Development-only traffic C++ DEBUG trace; never a production logger default')
    parser.add_argument('--required-green-observations', type=int, choices=(1,2,3))
    parser.add_argument('--unknown-grace-sec', type=float,
                        help='Guarded direct-observation trial only: pure UNKNOWN after authorized G, at most 0.3s')
    parser.add_argument('--stamped-vote-window-size', type=int, choices=(0,3,7))
    parser.add_argument('--stamped-vote-max-age-sec', type=float)
    parser.add_argument('--stamped-vote-startup-passthrough', choices=('true','false'))
    parser.add_argument('--record-video', action='store_true')
    parser.add_argument('--front-video-only', action='store_true',
                        help='Record actual annotated camera/telemetry only; do not capture any desktop rectangle')
    parser.add_argument('--video-label', choices=('current','confirm2','vote3','vote3_group','vote7','unknown_grace'))
    parser.add_argument('--video-display', default=os.environ.get('DISPLAY',':0'))
    parser.add_argument('--morai-window-id', help='Explicit Simulator X11 ID; only this rectangle is recorded')
    parser.add_argument('--video-encoder', choices=('libx264','h264_nvenc'), default='libx264')
    parser.add_argument('--stop-video-after-group', type=int, choices=(201326,),
                        help='Stop owned videos after a verified forward D crossing; bag/controller keep running')
    parser.add_argument('--video-tail-sec', type=float, default=5.)
    parser.add_argument('--native-python', type=Path,
                        default=Path.home()/'miniconda3/envs/traffic_light_gpu/bin/python')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    try:
        short_cycles = validate_short_cycle_request(args.experiment, args.short_cycle_plan,
                                                   args.mode, args.temporal_direct)
        if args.stop_video_after_group is not None:
            CrossingVideoDeadline('/unused', args.stop_video_after_group, args.video_tail_sec)
            if not args.record_video:
                raise ValueError('D video cut requires --record-video')
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    if args.preparation_geometry is not None and args.experiment not in ('stopped-yellow', 'first-observation-yellow', 'missing-signal'):
        parser.error('Separate preparation geometry applies only to stationary native-control trials')
    if args.yellow_durations_sec is not None and args.experiment not in ('stopped-yellow', 'first-observation-yellow', 'moving-native-yellow'):
        parser.error('Explicit yellow durations apply only to stopped-yellow or moving-native-yellow')
    moving_native = args.experiment == 'moving-native-yellow'
    if args.evaluation_speed_cap_mps is not None:
        from publish_traffic_braking_trial_speed_cap import validate_cap
        validate_cap(args.evaluation_speed_cap_mps)
        if args.mode == 'production':
            parser.error('Unmodified production trials cannot install an evaluation speed cap')
    if not math.isfinite(args.moving_red_hold_sec) or not 1 <= args.moving_red_hold_sec <= 20:
        parser.error('Moving red hold must be within 1..20 seconds')
    if moving_native and (args.yellow_durations_sec not in ((1.,), (2.,), (3.,)) or
                          not 1. <= args.trigger_front_distance_m <= 60.):
        parser.error('Moving native trial requires one --yellow-durations-sec 1|2|3 and trigger distance 1..60m')
    if args.planner_fixture and args.experiment != 'natural':
        parser.error('Planner-only fixtures cannot run with a native signal-control experiment')
    if args.mode == 'production' and (args.experiment != 'natural' or args.planner_fixture):
        parser.error('Unmodified production defaults require natural signals without a proxy')
    if args.mode == 'production' and (args.required_green_observations is not None or
                                      args.stamped_vote_window_size is not None or
                                      args.stamped_vote_max_age_sec is not None or
                                      args.stamped_vote_startup_passthrough is not None or
                                      args.unknown_grace_sec is not None):
        parser.error('Explicit temporal candidates require guarded mode, not production mode')
    if args.unknown_grace_sec is not None and (args.mode != 'guarded' or args.temporal_direct != 'true'):
        parser.error('UNKNOWN grace requires explicit guarded direct-observation mode')
    if args.stamped_vote_window_size in (3,7) and (args.mode != 'guarded' or args.temporal_direct != 'true'):
        parser.error('Stamped vote requires guarded mode with --temporal-direct true')
    try:
        temporal_video_launch_arguments('validation', 'only', {},
            stamped_vote=args.stamped_vote_window_size,
            stamped_vote_max_age_sec=args.stamped_vote_max_age_sec,
            stamped_vote_startup_passthrough=(None if args.stamped_vote_startup_passthrough is None
                                             else args.stamped_vote_startup_passthrough == 'true'),
            unknown_grace_sec=args.unknown_grace_sec)
    except ValueError as exc:
        parser.error(str(exc))
    try:
        validate_video_request(args.record_video, args.front_video_only, args.video_label, args.morai_window_id)
    except ValueError as exc:
        parser.error(str(exc))
    if args.visual_continuation_sec is not None and (
            args.mode == 'production' or not 0 <= args.visual_continuation_sec <= .3):
        parser.error('Visual continuation candidate requires explicit baseline/guarded mode and 0..0.3s')
    if args.perception_odometry_topic is not None and (
            args.mode == 'production' or
            not args.perception_odometry_topic.startswith('/evaluation/traffic_light_policy/') or
            any(ch.isspace() for ch in args.perception_odometry_topic)):
        parser.error('Stress odometry requires explicit baseline/guarded mode and an evaluation namespace')
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
    geometry_groups, _ = load_geometry(args.geometry)
    d_crossing_direction = next(group.get('crossing_direction', 'both') for group in geometry_groups
                               if group['group_id'] == 201326)
    if d_crossing_direction == 'both':
        # Historical audited D geometry's forward crossing orientation.
        d_crossing_direction = 'negative_to_positive'
    candidate_map_sha = None
    if args.map_file is not None:
        args.map_file = args.map_file.resolve(strict=True)
        candidate_map_sha = digest(args.map_file)
    if args.preparation_geometry is not None:
        load_geometry(args.preparation_geometry)
    screen_geometry = (simulator_capture_geometry(args.morai_window_id,args.video_display)
                       if args.record_video and not args.front_video_only else None)
    production = args.mode == 'production'
    guarded = None if production else args.mode == 'guarded'
    progress = None if production else args.route_progress == 'true'
    fixture = args.planner_fixture is not None
    native_signal_control = args.experiment != 'natural' and not fixture
    if args.observation_start_depth_m is not None and (args.mode == 'production' or not math.isfinite(args.observation_start_depth_m) or not 0 <= args.observation_start_depth_m <= 120 or args.temporal_direct != 'true'):
        parser.error('distance gate requires explicit guarded/baseline direct trial, finite depth 0..120')
    proxy = args.experiment not in ('natural', 'moving-native-yellow', 'first-observation-yellow', 'short-native-cycles') or fixture
    plugin = ROOT/'planning_ws/install_isolated/lib/libautoware_behavior_velocity_traffic_light_module.so'
    receipt = dict(mode=args.mode, route_progress=progress, experiment=args.experiment,
        production_policy_defaults_unmodified=production,
        planner_compliance_observations=None if production else args.temporal_direct=='true',
        visual_continuation_sec=args.visual_continuation_sec,
        perception_odometry_topic=args.perception_odometry_topic,
        enable_yellow_passthrough=None if production else args.yellow_passthrough=='true',
        signal_observation_timeout_sec=None if production else args.signal_timeout_sec,
        yellow_durations_sec=receipt_yellow_durations(args.experiment, args.yellow_durations_sec),
        group_id=args.group_id, source_config=str(args.config.resolve()),
        planner_fixture=args.planner_fixture, planner_fixture_is_synthetic=fixture,
        fixture_probe_seconds=args.fixture_probe_seconds if fixture else None,
        native_signal_SET_enabled=native_signal_control,
        native_signal_program_stress=short_cycles,
        passive_runtime_timing_enabled=True,
        natural_signal_program=not native_signal_control and not fixture,
        short_cycle_plan=(str(args.short_cycle_plan.resolve()) if short_cycles else None),
        short_cycle_plan_sha256=(digest(args.short_cycle_plan) if short_cycles else None),
        real_native_moving_yellow=moving_native,
        real_native_first_observation_yellow=args.experiment == 'first-observation-yellow',
        synthetic_canonical_signal_proxy=proxy,
        observation_start_depth_m=args.observation_start_depth_m,
        trigger_front_distance_m=args.trigger_front_distance_m if moving_native else None,
        trigger_distance_basis=getattr(args, 'trigger_distance_basis', 'finite_front'),
        moving_red_hold_sec=getattr(args, 'moving_red_hold_sec', 3.),
        evaluation_speed_cap_mps=getattr(args, 'evaluation_speed_cap_mps', None),
        perception_qualification=not fixture,
        compliance_debug_enabled=args.compliance_debug,
        required_green_observations=args.required_green_observations,
        signal_compliance_unknown_grace_sec=(args.unknown_grace_sec if production or
            args.unknown_grace_sec is not None else 0.0),
        stamped_vote_window_size=args.stamped_vote_window_size,
        stamped_vote_max_age_sec=args.stamped_vote_max_age_sec,
        stamped_vote_startup_passthrough=(None if args.stamped_vote_startup_passthrough is None
                                         else args.stamped_vote_startup_passthrough == 'true'),
        video=dict(enabled=args.record_video,label=args.video_label,encoder=args.video_encoder,
                   front_only=args.front_video_only,
                   simulator_rectangle=screen_geometry,
                   source=('stamped current annotated front camera only' if args.front_video_only else
                           'actual Simulator screen + stamped current annotated front camera'),
                   synthetic_perception_faults=False),
        installed_traffic_plugin_sha256=digest(plugin) if plugin.is_file() else None,
        config_sha256=digest(args.config), geometry_sha256=digest(args.geometry),
        preparation_geometry=(str(args.preparation_geometry) if args.preparation_geometry else None),
        preparation_geometry_sha256=(digest(args.preparation_geometry) if args.preparation_geometry else None),
        wrapper_sha256=digest(__file__), native_python=str(args.native_python),
        front_axle_offset_m=3.0,
        frame_provenance='configs/planning/vehicle_info.yaml and vehicle/README.md rear-axle base_link',
        resume_allowed=False, native_GT_is_planner_input=False,
        planner='real ROS1 planner/controller through unmodified validator runner')
    receipt['map_candidate'] = (dict(path=str(args.map_file), sha256=candidate_map_sha)
                                if args.map_file is not None else None)
    receipt['video_cut'] = dict(group_id=args.stop_video_after_group,
        tail_sec=args.video_tail_sec, crossing_direction=d_crossing_direction,
        driving_continues_to_safe_goal=True)
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
    debug_config = args.output/'traffic_cpp_debug.conf' if args.compliance_debug else None
    if debug_config is not None:
        debug_config.write_text('log4j.logger.ros=INFO\n'
            'log4j.logger.ros.autoware_behavior_velocity_traffic_light_module=DEBUG\n')
        receipt['compliance_debug_config_sha256'] = digest(debug_config)
    (args.output/'selection.json').write_text(json.dumps(receipt, indent=2)+'\n')
    lock, done, fatal = threading.Lock(), threading.Event(), threading.Event()
    phase_teardown = threading.Event()
    children, handles, subscriptions = [], [], []
    video_recorders, intentionally_stopped_video = [], set()
    video_cut_started = threading.Event()
    video_cut = (CrossingVideoDeadline(args.output/'observer/observations.jsonl',
                                      args.stop_video_after_group, args.video_tail_sec,
                                      d_crossing_direction)
                 if args.stop_video_after_group is not None else None)
    state = dict(mode='none', group_id=args.group_id, receipt_monotonic=0., received=False, terminal=False)
    fixture_state = dict(received=False, started_monotonic=time.monotonic(),
                         receipt_monotonic=0., handoff_real=False, error=False)

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
    original_recording = runner.Recording

    class TrialRecording(original_recording):
        def __init__(self, *recording_args, **recording_kwargs):
            super().__init__(*recording_args, **recording_kwargs)
            self.video_lock = threading.Lock()

        def start(self, basename):
            super().start(basename)
            video_recorders.append(self)

        def stop_video_only(self):
            from planning_scenario_validator.runtime import stop_group
            with self.video_lock:
                if self.video is not None and self.video.poll() is None:
                    stop_group(self.video, timeout=15)
                    event(dict(event='validator_screen_video_cut', path=str(self.video_path)))

        def stop(self, final_basename):
            with self.video_lock:
                return super().stop(final_basename)

    def cut_owned_videos():
        try:
            event(dict(event='D_video_cut_started', crossing=video_cut.trigger,
                       controller_and_bag_continue=True))
            stopped = []
            for name, process in children:
                if name in ('front_video', 'simulator_video') and process.poll() is None:
                    intentionally_stopped_video.add(name)
                    process.send_signal(signal.SIGTERM if name == 'front_video' else signal.SIGINT)
                    stopped.append((name, process))
            for recorder in video_recorders:
                recorder.stop_video_only()
            for name, process in stopped:
                process.wait(timeout=owned_child_exit_grace_sec(name))
                if name == 'front_video' and process.returncode != 0:
                    raise RuntimeError('Front video failed to finalize: '+str(process.returncode))
                event(dict(event='owned_video_cut', name=name, returncode=process.returncode))
            event(dict(event='D_video_cut_complete', controller_and_bag_continue=True))
        except Exception as error:
            event(dict(event='D_video_cut_error', error=repr(error)))
            fatal.set()
    proxy_started = False
    publisher = None
    def selected_launch(package, filename, **kwargs):
        nonlocal proxy_started, publisher
        if args.map_file is not None and digest(args.map_file) != candidate_map_sha:
            raise RuntimeError('Map candidate changed after qualification')
        selected = launch_arguments(package, filename, kwargs, guarded, progress, proxy,
            temporal_direct=args.temporal_direct=='true', signal_timeout_sec=args.signal_timeout_sec,
            yellow_passthrough=args.yellow_passthrough=='true',
            visual_continuation_sec=args.visual_continuation_sec,
            perception_odometry_topic=args.perception_odometry_topic)
        selected = map_launch_arguments(package, filename, selected, args.map_file, candidate_map_sha)
        if args.observation_start_depth_m is not None and package == 'traffic_light_detection' and filename in ('traffic_light_node.launch', 'traffic_light_v5_candidate.launch'):
            selected['observation_start_depth_m'] = args.observation_start_depth_m
        selected = temporal_video_launch_arguments(package,filename,selected,
            args.required_green_observations,args.stamped_vote_window_size,args.record_video,
            stamped_vote_max_age_sec=args.stamped_vote_max_age_sec,
            stamped_vote_startup_passthrough=(None if args.stamped_vote_startup_passthrough is None
                                             else args.stamped_vote_startup_passthrough == 'true'),
            unknown_grace_sec=args.unknown_grace_sec)
        if proxy and package == 'traffic_light_detection' and not proxy_started:
            import rospy
            from autoware_perception_msgs.msg import TrafficLightGroupArray
            from std_msgs.msg import String
            publisher = rospy.Publisher(CANONICAL, TrafficLightGroupArray, queue_size=1)
            if fixture:
                # The runner initializes rospy during stack setup, after the
                # helper starts. Start its receive allowance when subscriptions
                # can actually exist, not during preceding ROS/model preflight.
                with lock:
                    fixture_state['started_monotonic'] = time.monotonic()
                def fixture_status(message):
                    try:
                        body = json.loads(message.data)
                        accepted = validated_fixture_status(body, args.group_id, args.planner_fixture)
                        with lock:
                            fixture_state.update(accepted, received=True,
                                                 receipt_monotonic=time.monotonic())
                        event(dict(event='planner_fixture_status', status=body))
                    except Exception as error:
                        fatal.set()
                        event(dict(event='planner_fixture_status_error', error=str(error)))

                def fixture_forward(message):
                    try:
                        with lock:
                            relay = fixture_relay_mode(fixture_state, time.monotonic())
                        if relay != 'fixture' or phase_teardown.is_set():
                            return
                        ids = [int(group.traffic_light_group_id) for group in message.traffic_light_groups]
                        if ids != [args.group_id]:
                            raise ValueError('Fixture wire message must contain only its explicit mission group')
                        publisher.publish(message)
                        event(dict(event='planner_fixture_relay', planner_only=True, synthetic=True,
                            source_stamp_ns=int(message.stamp.sec)*10**9+int(message.stamp.nanosec),
                            output_ids=ids))
                    except Exception as error:
                        fatal.set()
                        event(dict(event='planner_fixture_signal_error', error=str(error)))

                def real_after_fixture(message):
                    try:
                        with lock:
                            relay = fixture_relay_mode(fixture_state, time.monotonic())
                        if relay == 'real' and not phase_teardown.is_set():
                            publisher.publish(message)
                    except Exception as error:
                        fatal.set()
                        event(dict(event='planner_fixture_handoff_error', error=str(error)))
                subscriptions.extend((rospy.Subscriber(FIXTURE_STATUS_TOPIC, String, fixture_status, queue_size=1),
                                      rospy.Subscriber(FIXTURE_TOPIC, TrafficLightGroupArray, fixture_forward, queue_size=1),
                                      rospy.Subscriber(SOURCE_TOPIC, TrafficLightGroupArray, real_after_fixture, queue_size=1)))
                proxy_started = True
                command = original_launch(package, filename, **selected)
                event(dict(event='launch', package=package, filename=filename, command=command))
                return command
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
        command = with_compliance_debug(original_launch(package, filename, **selected),
                                        package, filename, debug_config)
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
                    process.wait(timeout=owned_child_exit_grace_sec(name))
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
            if args.evaluation_speed_cap_mps is not None:
                start('evaluation_speed_cap', ['/usr/bin/python3',
                    str(ROOT/'scripts/evaluation/publish_traffic_braking_trial_speed_cap.py'),
                    '--max-velocity-mps', str(args.evaluation_speed_cap_mps),
                    '--max-seconds', str(duration), '--output', str(args.output/'speed_cap')])
            start('runtime_timing', ['/usr/bin/python3',
                str(ROOT/'scripts/evaluation/observe_traffic_light_runtime_timing.py'),
                '--duration-sec', str(duration), '--output-dir', str(args.output/'runtime_timing')])
            if args.record_video:
                if not args.front_video_only:
                    activate_simulator_for_capture(args.morai_window_id,args.video_display)
                    current_geometry=simulator_capture_geometry(args.morai_window_id,args.video_display)
                    if current_geometry != screen_geometry:
                        raise RuntimeError('Simulator window moved after video preflight')
                    g=screen_geometry
                    codec=(['-c:v','h264_nvenc','-preset','fast','-b:v','2500k']
                           if args.video_encoder=='h264_nvenc' else
                           ['-c:v','libx264','-preset','ultrafast','-crf','28','-threads','2'])
                    start('simulator_video',['ffmpeg','-nostdin','-loglevel','warning','-n',
                        '-f','x11grab','-framerate','10','-video_size',f"{g['width']}x{g['height']}",
                        '-draw_mouse','0','-i',f"{args.video_display}+{g['x']},{g['y']}",
                        '-t',str(duration),*codec,'-pix_fmt','yuv420p',
                        str(args.output/'simulator.mp4')])
                start('front_video',['/usr/bin/python3',str(ROOT/'scripts/evaluation/record_traffic_light_evidence_video.py'),
                    '--output',str(args.output/'front_video'),'--label',args.video_label,
                    '--fps','10','--max-seconds',str(duration),'--encoder',args.video_encoder])
            start('observer', [str(args.native_python), str(ROOT/'scripts/evaluation/observe_traffic_light_compliance.py'),
                '--output', str(args.output/'observer'), '--geometry', str(args.geometry), '--duration-sec',str(duration),
                '--crossing-jpegs'])
            if fixture:
                with lock:
                    fixture_state['started_monotonic'] = time.monotonic()
                start('phase_controller', [str(args.native_python), str(ROOT/'scripts/evaluation/synth_traffic_light_planner_trial.py'),
                    '--output', str(args.output/'planner_fixture'), '--geometry', str(args.geometry),
                    '--group-id', str(args.group_id), '--mode', args.planner_fixture,
                    '--probe-seconds', str(args.fixture_probe_seconds), '--max-seconds', str(duration)])
            elif short_cycles:
                if digest(args.short_cycle_plan) != receipt['short_cycle_plan_sha256']:
                    raise RuntimeError('Short-cycle plan changed after qualification')
                start('phase_controller', [str(args.native_python),
                    str(ROOT/'scripts/evaluation/control_traffic_light_short_cycles.py'),
                    '--plan', str(args.short_cycle_plan), '--output',
                    str(args.output/'phase_controller'), '--max-seconds', str(duration)])
            elif native_signal_control:
                phase_script = ('control_traffic_light_green_loss_trial.py' if args.experiment=='green-loss'
                                else 'control_traffic_light_policy_trial.py')
                phase_command = [str(args.native_python), str(ROOT/'scripts/evaluation'/phase_script),
                    '--output',str(args.output/'phase_controller'),'--geometry',str(args.preparation_geometry or args.geometry),
                    '--group-id',str(args.group_id),'--experiment',args.experiment,
                    '--max-seconds', str(min(600, duration))]
                if args.yellow_durations_sec is not None:
                    phase_command.extend(('--yellow-durations-sec',
                        ','.join(format(value, 'g') for value in args.yellow_durations_sec)))
                if moving_native:
                    phase_command.extend(('--trigger-front-distance-m', str(args.trigger_front_distance_m)))
                    phase_command.extend(('--trigger-distance-basis', getattr(args, 'trigger_distance_basis', 'finite_front'),
                        '--moving-red-hold-sec', str(getattr(args, 'moving_red_hold_sec', 3.))))
                start('phase_controller', phase_command)
            def watch():
                while not done.wait(.2):
                    try:
                        if video_cut is not None and not video_cut_started.is_set() and video_cut.poll(time.monotonic()):
                            video_cut_started.set()
                            threading.Thread(target=cut_owned_videos, daemon=True).start()
                        with lock:
                            if not phase_teardown.is_set():
                                if fixture and proxy_started:
                                    fixture_relay_mode(fixture_state, time.monotonic())
                                elif not fixture:
                                    active_fault_mode(state, time.monotonic())
                    except (RuntimeError, ValueError, OSError) as error:
                        event(dict(event='trial_watch_error', error=repr(error)))
                        fatal.set()
                    if fatal.is_set() and not phase_teardown.is_set():
                        event(dict(event='proxy_fatal_abort'))
                        os.kill(os.getpid(), signal.SIGTERM)
                        return
                    for name, process in children:
                        code = process.poll()
                        if name in intentionally_stopped_video:
                            continue
                        if (code is not None and (code != 0 or name == 'observer' or fixture) and
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
        runner.Recording = TrialRecording
        code = runner.main([str(args.config), '--resume','false'])
        event(dict(event='validator_returned', returncode=code))
        return code
    finally:
        done.set()
        runner._launch = original_launch
        runner._stop_and_verify = original_stop
        runner.LiveMonitor = original_monitor
        runner._run_campaign = original_campaign
        runner.Recording = original_recording
        close_helpers()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
