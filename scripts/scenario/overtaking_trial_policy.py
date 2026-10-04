#!/usr/bin/env python3
"""Opt-in MORAI experiment only. Requests UUID-bound safe leases, never forced activation.

Does not publish controls, change routes, or cancel running maneuvers. Launch only
with the matching one-shot RTC implementation and a verified v85 Route B.
"""
import argparse
import hashlib
import json
import math
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path


def path_aligned_leads(points, ego, objects, ego_width):
    """Request-trigger geometry, NOT an RTC/Cruise safety or collision decision.

    Follow the *published* BPP path rather than the current lane's successor IDs.
    An object already overlapping its swept lateral strip, or predicted to enter
    that strip, may warrant looking for a safe passing candidate. No extrapolation
    beyond path ends, no extra lateral margin, and no crossing/oncoming objects.
    Returned gap is current center-to-center planar path distance (not TTC).
    """
    if not math.isfinite(ego_width) or ego_width <= 0 or len(points) < 2:
        raise ValueError('invalid_lead_path_geometry')
    if not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('nonfinite_lead_path')
    segments, arc = [], 0.
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length > 1e-8:
            segments.append((a[0], a[1], dx/length, dy/length, length, arc))
            arc += length
    if not segments:
        raise ValueError('empty_lead_path')

    def project_pose(pose):
        x, y, heading = pose
        if not all(math.isfinite(v) for v in pose):
            return None
        best = None
        for i, (px, py, ux, uy, length, start) in enumerate(segments):
            along = (x-px)*ux + (y-py)*uy
            clipped = min(length, max(0., along))
            distance = math.hypot(x-px-clipped*ux, y-py-clipped*uy)
            delta = math.atan2(math.sin(heading-math.atan2(uy, ux)),
                               math.cos(heading-math.atan2(uy, ux)))
            candidate = (distance, start+clipped, delta,
                         (i == 0 and along < 0) or (i == len(segments)-1 and along > length))
            if best is None or candidate[0] < best[0]:
                best = candidate
        return None if best is None or best[3] else best[:3]

    ego_projection = project_pose(ego)
    if (ego_projection is None or ego_projection[0] > ego_width/2 or
            abs(ego_projection[2]) >= math.pi/4):
        raise ValueError('ego_outside_lead_path')
    leads = []
    for obj in objects:
        length, width = obj['length'], obj['width']
        if not all(math.isfinite(v) and v > 0 for v in (length, width)):
            continue
        current = project_pose(obj['pose'])
        if current is None or current[1] <= ego_projection[1] or abs(current[2]) >= math.pi/4:
            continue
        for stamp, pose in [(0., obj['pose'])] + obj.get('predictions', []):
            if not math.isfinite(stamp) or stamp < 0:
                continue
            projected = current if stamp == 0 else project_pose(pose)
            if projected is None or projected[1] <= ego_projection[1] or abs(projected[2]) >= math.pi/4:
                continue
            radius = abs(math.sin(projected[2]))*length/2 + abs(math.cos(projected[2]))*width/2
            if projected[0] <= ego_width/2 + radius:
                leads.append(dict(uuid=obj['uuid'], gap=current[1]-ego_projection[1],
                    speed=obj['speed'], length=length, path_lateral=current[0],
                    path_match_time=stamp, lead_source='published_bpp_path'))
                break
    return sorted(leads, key=lambda o: o['gap'])


def forward_lane_chain(lane, successors, max_hops=20):
    """Follow unambiguous physical continuations, never an adjacent lane/fork."""
    chain = {lane}
    for _ in range(max_hops):
        choices = successors.get(lane, [])
        if len(choices) != 1 or choices[0] in chain:
            break
        lane = choices[0]
        chain.add(lane)
    return chain


def authoritative_inactive(snapshot, now, route_uuid, route_stamp, modules):
    """No inference from missing debug data: require complete fresh manager data."""
    if (snapshot.get('valid') is not True or snapshot.get('complete_snapshot') is not True or
            snapshot.get('route_uuid') != route_uuid or
            not isinstance(snapshot.get('route_stamp'), (float, int)) or
            abs(snapshot['route_stamp']-route_stamp) > 1e-6 or
            not isinstance(snapshot.get('stamp'), (float, int)) or
            not 0 <= now-snapshot['stamp'] <= .4 or
            not isinstance(snapshot.get('modules'), list)):
        return None
    active = set()
    active_uuids = {name:[] for name in modules}
    for module in snapshot['modules']:
        if (not isinstance(module.get('active'), bool) or not isinstance(module.get('name'), str)
                or not isinstance(module.get('approved'), bool)
                or not isinstance(module.get('waiting_approval'), bool)
                or module.get('state') not in (0, 1, 2, 3, 4)
                or module.get('rtc_uuid_available') is not True or not isinstance(module.get('rtc_uuids'), list)):
            return None
        physical_owner = ((module['approved'] and module['state'] not in (3, 4)) or
                          (module['state'] == 1 and not module['waiting_approval']))
        if module['active'] != physical_owner:
            return None  # Reject older/inconsistent publisher ownership semantics.
        if module['active']:
            active.add(module['name'])
            for rtc in module['rtc_uuids']:
                if not isinstance(rtc.get('uuid'), str) or not isinstance(rtc.get('interface'), str):
                    return None
                names = {module['name'],rtc['interface'],module['name']+'_'+rtc['interface']}
                for name in names.intersection(modules):
                    active.add(name)
                    active_uuids[name].append(rtc['uuid'])
            if 'lane_change' in module['name'] and module['name'] not in modules:
                # Unknown name mapping is not evidence that all LC owners are idle.
                active.update(modules)
    return {name:dict(active=name in active, stamp=snapshot['stamp'],
                     active_uuids=active_uuids[name], uuid_complete=True) for name in modules}


def ownership_busy(statuses, now, inactive=None):
    """Stale RUNNING is unresolved unless explicit fresh inactive evidence exists.

    inactive is reserved for an authoritative lifecycle adapter, not empty-path
    inference. Existing runtime has no such publisher, so default stays blocked.
    """
    unresolved = []
    inactive = inactive or {}
    for module, entries in statuses.items():
        for entry in entries:
            if entry['state'] not in (1, 2):
                continue
            age = now-entry['stamp']
            evidence = inactive.get(module)
            confirmed = (evidence is not None and
                         (evidence.get('active') is False or
                          (evidence.get('uuid_complete') and entry['uuid'] not in evidence.get('active_uuids', []))) and
                         0 <= now-evidence['stamp'] <= .4 and
                         evidence['stamp'] > entry['stamp'])
            if age <= .4 or not confirmed:
                unresolved.append(dict(module=module, uuid=entry['uuid'], age_sec=age,
                                       status='fresh_running' if age <= .4 else 'stale_unresolved'))
    return bool(unresolved), unresolved


def verified_route_status(status, route_stamp):
    """A latched verification is valid only for this route publication lifecycle."""
    return (status.get('phase') == 'VERIFIED' and status.get('verified_active') == 'route_b'
            and status.get('route_key') == 'route_b' and status.get('uncertain') is False
            and status.get('busy') is False and isinstance(status.get('generation'), int)
            and isinstance(status.get('stamp'), (int, float))
            and status['stamp'] >= route_stamp - .05)


class Policy:
    """Pure state machine; all geometry/freshness evidence is supplied by adapter."""
    def __init__(self):
        self.state = 'IDLE'
        self.lead = None
        self.candidate = None
        self.slow_since = None
        self.owner_missing_since = None
        self.reason = 'initializing'

    def step(self, s):
        self.reason = 'observe'
        if not s['valid']:
            self.slow_since = None
            self.reason = s.get('reason', 'invalid_inputs')
            return None
        if self.candidate is not None:
            bound = [r['uuid'] for values in s['rtc'].values() for r in values if r.get('auto_mode')]
            if any(candidate != self.candidate for candidate in bound):
                self.state = 'HOLD'
                self.reason = 'server_bound_different_uuid_no_further_requests'
                return None
        direction = 'left' if self.state in ('WAIT_LEFT', 'LEFT_RUNNING') else 'right'
        statuses = s['rtc'].get(direction, [])
        owned = next((r for r in statuses if r['uuid'] == self.candidate), None)
        if self.state in ('WAIT_LEFT', 'WAIT_RIGHT', 'LEFT_RUNNING', 'RIGHT_RUNNING') and owned:
            if owned['state'] == 3:
                self.state = 'PASSING' if direction == 'left' else 'DONE'
                self.candidate = None
                self.owner_missing_since = None
                self.reason = 'observed_owner_success'
                return None
            if owned['state'] == 4:
                self.state = 'HOLD'
                self.reason = 'observed_owner_failure'
                return None
        if self.state.endswith('_RUNNING'):
            # Completion monitoring only; no service call while maneuver runs.
            if owned:
                self.owner_missing_since = None
            elif self.owner_missing_since is None:
                self.owner_missing_since = s['now']
            elif s['now'] - self.owner_missing_since >= 2.0:
                self.state = 'HOLD'
                self.reason = 'running_owner_missing_unresolved'
                return None
            self.reason = 'running_owner_or_completion'
            return None
        if self.state in ('WAIT_LEFT', 'WAIT_RIGHT') and owned and owned['state'] in (1, 2):
            self.state = 'LEFT_RUNNING' if direction == 'left' else 'RIGHT_RUNNING'
            return None
        if s['busy']:
            self.reason = 'another_lane_change_running'
            return None
        if self.state in ('HOLD', 'DONE'):
            self.reason = self.state.lower()
            return None
        if self.state == 'IDLE':
            lead = s.get('lead')
            eligible = (s['remaining'] > 150 and s['left_allowed'] and lead and
                        8 < lead['gap'] < 100 and s['desired_speed'] - lead['speed'] >= 3 and
                        s['ego_speed'] >= 8)
            if not eligible:
                self.slow_since = None
                self.reason = 'no_persistent_slow_lead_or_no_pass_reserve'
                return None
            if self.lead != lead['uuid']:
                self.lead = lead['uuid']
                self.slow_since = s['now']
            if self.slow_since is None:
                self.slow_since = s['now']
            if s['now'] - self.slow_since < 1.0:
                self.reason = 'slow_lead_confirmation'
                return None
            direction = 'left'
        elif self.state == 'PASSING':
            if not s.get('tracked_lead_present'):
                self.reason = 'passed_lead_missing_no_return_request'
                return None
            if s['remaining'] <= 80:
                self.state = 'HOLD'
                self.reason = 'mandatory_route_return_reserve'
                return None
            if not s.get('fully_passed') or not s['right_allowed']:
                self.reason = 'not_fully_passed_or_no_right_lane'
                return None
            direction = 'right'
        else:
            if not owned or owned['state'] == 4:
                self.state = 'HOLD'
                self.reason = 'candidate_disappeared_or_failed'
                return None
            if direction == 'left' and (s['remaining'] <= 150 or not s['left_allowed'] or
                    not s.get('lead') or s['lead']['uuid'] != self.lead):
                self.reason = 'pass_guard_changed_lease_not_renewed'
                return None
            if direction == 'right' and (not s.get('fully_passed') or not s['right_allowed'] or s['remaining'] <= 80):
                self.reason = 'return_guard_changed_lease_not_renewed'
                return None
        candidates = [r for r in s['rtc'][direction] if r['state'] == 0 and r['safe']]
        # Also reject unsafe WAITING peers: server requires a sole fresh candidate.
        waiting = [r for r in s['rtc'][direction] if r['state'] == 0]
        if len(candidates) != 1 or len(waiting) != 1:
            self.reason = 'no_unique_safe_waiting_candidate'
            return None
        chosen = candidates[0]['uuid']
        if self.candidate is not None and chosen != self.candidate:
            self.state = 'HOLD'
            self.reason = 'candidate_uuid_changed'
            return None
        self.candidate = chosen
        self.state = 'WAIT_LEFT' if direction == 'left' else 'WAIT_RIGHT'
        self.reason = 'request_one_shot_auto_lease'
        return direction


class PassingPolicy:
    """Bidirectional outbound with ACK; normal planner owns preferred return."""
    def __init__(self):
        self.state='IDLE';self.reason='initializing';self.lead=None;self.candidate=None
        self.direction=None;self.slow_since=None;self.intent_id=None
        self.source_chain=[];self.target_chain=[];self.owner_missing_since=None
        self.intent_route=None;self.intent_boot=None;self.return_started=None

    def step(self,s):
        self.reason='observe'
        if not s['valid']:
            self.slow_since=None;self.reason=s.get('reason','invalid_inputs');return None
        if self.state in ('HOLD','DONE'):
            self.reason=self.state.lower();return None
        if self.state=='RETURN_RELEASED':
            if s['lane'] in self.source_chain and not s['busy']:
                self.state='DONE';self.reason='normal_return_observed'
            else:self.reason='normal_planner_owns_return'
            return None
        if self.state=='PASSING':
            if not s.get('tracked_lead_present') or s['remaining']<=max(150,s.get('ack_reserve',150)):
                self.state='RETURN_RELEASED';self.reason='release_for_missing_lead_or_mandatory_return'
            elif s.get('fully_passed'):
                self.state='RETURN_RELEASED';self.reason='fully_passed_release_preferred_return'
            else:self.reason='hold_pass_lane'
            return None
        owned=next((r for r in s['rtc'].get(self.direction,[]) if r['uuid']==self.candidate),None)
        if self.state in ('ARM','OUTBOUND'):
            if any(r.get('auto_mode') and r['uuid']!=self.candidate for values in s['rtc'].values() for r in values):
                self.state='HOLD';self.reason='server_bound_other_uuid';return None
            if owned and owned['state']==3:
                self.state='PASSING';self.reason='observed_outbound_success';return None
            if owned and owned['state']==4:
                self.state='HOLD';self.reason='outbound_failed';return None
            if owned and owned['state'] in (1,2):
                self.state='OUTBOUND';self.owner_missing_since=None;self.reason='outbound_running';return None
            if self.state=='OUTBOUND':
                if self.owner_missing_since is None:self.owner_missing_since=s['now']
                if s['now']-self.owner_missing_since>=2:
                    self.state='HOLD';self.reason='outbound_completion_missing'
                return None
            if (s['busy'] or not s.get('lead') or s['lead']['uuid']!=self.lead or
                    s['remaining']<=150 or not s[self.direction+'_allowed']):
                self.state='HOLD';self.reason='arm_guard_changed';return None
            waiting=[r for r in s['rtc'][self.direction] if r['state']==0]
            if len(waiting)!=1 or waiting[0]['uuid']!=self.candidate or not waiting[0]['safe']:
                self.state='HOLD';self.reason='candidate_changed_or_unsafe';return None
            if not s.get('intent_ack'):
                self.reason='waiting_bpp_intent_ack';return None
            self.reason='request_one_shot_auto_lease';return self.direction
        if s['busy']:
            self.reason='another_lane_change_running';self.slow_since=None;return None
        lead=s.get('lead')
        if not (s['remaining']>150 and lead and 8<lead['gap']<100 and
                s['desired_speed']-lead['speed']>=3 and s['ego_speed']>=8):
            self.slow_since=None;self.reason='no_persistent_slow_lead_or_no_pass_reserve';return None
        if self.lead!=lead['uuid'] or self.slow_since is None:
            self.lead=lead['uuid'];self.slow_since=s['now']
        if s['now']-self.slow_since<1:
            self.reason='slow_lead_confirmation';return None
        for direction in ('left','right'):
            waiting=[r for r in s['rtc'][direction] if r['state']==0]
            chains=s.get('paired_chains',{}).get(direction)
            if not s[direction+'_allowed'] or len(waiting)!=1 or not waiting[0]['safe'] or not chains:
                continue
            self.direction=direction;self.candidate=waiting[0]['uuid'];self.intent_id=uuid.uuid4().hex
            self.source_chain,self.target_chain=chains
            self.intent_route=s['intent_route'];self.intent_boot=s['intent_boot']
            self.state='ARM';self.reason='publish_intent_before_auto_request';return None
        self.reason='no_unique_safe_bidirectional_candidate';return None

    def intent(self,s,stamp,map_hash):
        if not s.get('valid') or self.state not in ('ARM','OUTBOUND','PASSING'):
            return None
        # Guard changes stop renewal; never cancel an owned running maneuver.
        if not s.get('tracked_lead_present') or s['remaining']<=150:return None
        return dict(intent_uuid=self.intent_id,outbound_uuid=self.candidate,lead_uuid=self.lead,
            route_uuid=self.intent_route[0],route_stamp=self.intent_route[1],boot_id=self.intent_boot,
            map_sha=map_hash,direction=self.direction,phase=self.state,stamp=stamp,ttl=.5,
            source_chain=self.source_chain,target_chain=self.target_chain)


class RetryPassingPolicy(PassingPolicy):
    """Simulator retry wrapper; never infer that an ambiguous grant was released.

    The underlying candidate/ACK/atomic lease gates and normal-planner return
    ownership are unchanged. Retired UUIDs stay retired for this process/route.
    """
    MODULES = tuple(prefix+direction for prefix in
        ('lane_change_', 'avoidance_by_lane_change_', 'external_request_lane_change_')
        for direction in ('left', 'right'))
    RECOVERABLE = frozenset(('outbound_failed', 'arm_guard_changed',
        'candidate_changed_or_unsafe', 'atomic_uuid_lease_rejected', 'normal_return_observed'))
    COOLDOWN_SEC = 2.0  # Server lease <=1 s; not a substitute for inactive proof.
    SETTLED_SEC = .4

    def __init__(self):
        super().__init__()
        self.retired = set()
        self.lifecycle = None
        self.attempt_count = 0
        self.outcome_reason = None
        self.retry_after = None
        self.settle_stamp = None
        self.last_owner_stamp = None
        self.last_owner_sequence = None

    def _reset_settle(self):
        self.settle_stamp = self.last_owner_stamp = self.last_owner_sequence = None

    def _finish(self, reason, now):
        self.outcome_reason = reason
        if self.candidate:
            self.retired.add(self.candidate)
        self._reset_settle()
        self.slow_since = None
        if reason in self.RECOVERABLE:
            self.state = 'COOLDOWN'
            self.retry_after = now+self.COOLDOWN_SEC
        else:
            self.state = 'HOLD'
        self.reason = reason

    def record_rpc_result(self, s, result):
        # Only an exact, well-formed negative response is a known rejection.
        # A transport exception or malformed/mismatched response may have bound
        # a lease: preserve HOLD until an operator starts a new policy process.
        if result is True:
            # An acknowledged lease already has an owner even if the next RTC
            # snapshot still says WAITING. Never send a second grant RPC while
            # awaiting RUNNING/terminal; absence is ambiguity, not rejection.
            self.state='OUTBOUND';self.reason='atomic_uuid_lease_granted'
            self.owner_missing_since=s['now']
        elif result is False:
            self._finish('atomic_uuid_lease_rejected', s['now'])
        elif result is not True:
            self._finish('atomic_uuid_lease_ambiguous', s['now'])

    def _settled(self, s):
        owners = s.get('authoritative_owners', {})
        now = s.get('ros_now', float('nan'))
        sequence = s.get('ownership_sequence')
        if (s.get('busy') is not False or s.get('ownership_evidence') or
                s.get('rtc_auto_owners') is None or s['rtc_auto_owners'] or
                set(owners) != set(self.MODULES) or not isinstance(sequence, int)):
            self._reset_settle();return False
        stamps = []
        for owner in owners.values():
            stamp = owner.get('stamp', float('nan'))
            if (owner.get('active') is not False or owner.get('uuid_complete') is not True or
                    owner.get('active_uuids') != [] or not isinstance(stamp, (float,int)) or
                    not 0 <= now-stamp <= .4):
                self._reset_settle();return False
            stamps.append(stamp)
        if max(stamps) != min(stamps):
            self._reset_settle();return False
        stamp = stamps[0]
        if self.last_owner_stamp is not None:
            if stamp == self.last_owner_stamp and sequence == self.last_owner_sequence:
                return False  # A cached snapshot never advances settled time.
            if stamp <= self.last_owner_stamp or sequence <= self.last_owner_sequence:
                self._reset_settle();return False
            if stamp-self.last_owner_stamp > .4:
                self._reset_settle()
        if self.settle_stamp is None:
            self.settle_stamp = stamp
        self.last_owner_stamp, self.last_owner_sequence = stamp, sequence
        return stamp-self.settle_stamp >= self.SETTLED_SEC-1e-9

    def step(self, s):
        if self.state == 'HOLD':
            self.reason = self.outcome_reason or self.reason
            return None
        if not s.get('valid'):
            self._reset_settle();self.slow_since = None
            self.reason = s.get('reason', 'invalid_inputs')
            if self.reason in ('route_lifecycle_changed_restart_policy_required',
                    'ownership_epoch_changed_restart_policy_required', 'map_file_changed_restart_required'):
                self._finish(self.reason, s['now'])
            return None
        lifecycle = (s.get('intent_route'), s.get('intent_boot'))
        if self.lifecycle is None:
            self.lifecycle = lifecycle
        elif self.lifecycle != lifecycle:
            self._finish('retry_route_or_boot_changed', s['now']);return None
        if self.state == 'COOLDOWN':
            settled = self._settled(s)
            if s['remaining'] <= max(150, s.get('ack_reserve',150)) or s.get('mandatory_priority'):
                self.reason = 'retry_mandatory_route_priority';return None
            if s['now'] < self.retry_after:
                self.reason = 'retry_cooldown';return None
            if not settled:
                self.reason = 'retry_waiting_complete_inactive_ownership';return None
            # Keep tombstones, but require a new lead confirmation and new ACK.
            self.state = 'IDLE';self.lead = self.candidate = self.direction = None
            self.intent_id = self.intent_route = self.intent_boot = None
            self.source_chain=[];self.target_chain=[];self.owner_missing_since=None
            self.slow_since=None;self.reason='retry_rearmed';return None
        if self.state == 'IDLE':
            # Do not hide an old candidate to make a multiple-candidate set look
            # unique: disable that direction until its retired UUID disappears.
            if s.get('rtc_auto_owners'):
                self.slow_since=None;self.reason='existing_external_auto_lease';return None
            s = dict(s)
            for direction in ('left','right'):
                if any(r['uuid'] in self.retired for r in s['rtc'][direction] if r['state']==0):
                    s[direction+'_allowed'] = False
            if s.get('mandatory_priority') or s['remaining']<=max(150,s.get('ack_reserve',150)):
                self.slow_since=None;self.reason='mandatory_route_priority';return None
        previous = self.state
        request = super().step(s)
        if previous == 'IDLE' and self.state == 'ARM':
            self.attempt_count += 1
        if self.state in ('HOLD','DONE'):
            self._finish(self.reason, s['now'])
        return request


def atomic_response_outcome(candidate, module, responses):
    """True=bound, False=known rejection, None=ambiguous/malformed response."""
    if (len(responses)!=1 or responses[0].get('uuid')!=candidate or
            responses[0].get('module')!=module or
            not isinstance(responses[0].get('success'),bool)):
        return None
    return responses[0]['success']


def atomic_response_success(candidate, module, responses):
    return (len(responses)==1 and responses[0].get('uuid')==candidate and
            responses[0].get('module')==module and responses[0].get('success') is True)


def candidate_service_name(direction):
    if direction not in ('left','right'):
        raise ValueError('invalid_candidate_direction')
    # This endpoint does not exist on legacy force-command servers. No fallback.
    return '/planning/enable_auto_mode_candidate/external_request_lane_change_'+direction


def require_candidate_service_type(name, service_type):
    if (name not in (candidate_service_name('left'),candidate_service_name('right')) or
            service_type!='tier4_rtc_msgs/CooperateCommands'):
        raise RuntimeError('UUID candidate lease endpoint missing or incompatible: '+name)


def main():
    import rospy
    import rosservice
    import yaml
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from autoware_perception_msgs.msg import PredictedObjects
    from autoware_planning_msgs.msg import LaneletRoute
    from autoware_internal_planning_msgs.msg import PathWithLaneId
    from tier4_rtc_msgs.msg import CooperateStatusArray, CooperateCommand, Command, Module
    from tier4_rtc_msgs.srv import CooperateCommands, CooperateCommandsRequest
    from competition_route_requester.geometry import Geometry, project, inside

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enable-simulator-experiment', action='store_true', required=True)
    args = parser.parse_args()
    repo = next(p for p in Path(__file__).resolve().parents
                if (p/'base_ws/src/planning_addons/competition_route_requester/config/kcity_v85_competition_route.yaml').is_file())
    cfg = yaml.safe_load((repo / 'base_ws/src/planning_addons/competition_route_requester/config/kcity_v85_competition_route.yaml').read_text())
    vehicle = yaml.safe_load((repo / 'configs/planning/vehicle_info.yaml').read_text())
    ego_width = sum(float(vehicle[key]) for key in ('wheel_tread', 'left_overhang', 'right_overhang'))
    geometry = Geometry(repo, cfg)
    map_path = repo / cfg['map']['file']
    map_hash = hashlib.sha256(map_path.read_bytes()).hexdigest()
    if map_hash != '5111ae16ba788b059e40e88201b51b0cb4d1d662dcd8456f43cda9705066be58':
        raise RuntimeError('experiment supports exact v85 only')
    map_stat = (map_path.stat().st_mtime_ns, map_path.stat().st_size)
    relations = {}
    map_root = ET.parse(map_path).getroot()
    ways = {int(w.attrib['id']): [int(n.attrib['ref']) for n in w.findall('nd')]
            for w in map_root.findall('way')}
    for r in map_root.findall('relation'):
        tags = {t.attrib['k']: t.attrib['v'] for t in r.findall('tag')}
        if tags.get('type') == 'lanelet':
            relations[int(r.attrib['id'])] = (tags, {m.attrib['role']: int(m.attrib['ref']) for m in r.findall('member') if m.attrib['type'] == 'way'})
    rospy.init_node('overtaking_trial_policy')
    prefix = '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/'
    if not rospy.get_param(prefix+'lane_change/passing_intent/enable_simulator_experiment',False):
        raise RuntimeError('BPP passing-intent gate not opted in')
    modules = ['lane_change_left', 'lane_change_right', 'avoidance_by_lane_change_left',
               'avoidance_by_lane_change_right', 'external_request_lane_change_left', 'external_request_lane_change_right']
    for direction in ('left', 'right'):
        key = prefix + 'external_request_lane_change_' + direction
        if not rospy.get_param(key + '/auto_mode_single_candidate', False):
            raise RuntimeError('one-shot RTC server not enabled: ' + key)
        if not 0 < rospy.get_param(key + '/auto_mode_lease_sec', 0) <= 1.0:
            raise RuntimeError('RTC lease must be <= 1 second')
    latest = {}
    def receive(key, msg):
        latest[key] = (msg, time.monotonic())
    rospy.Subscriber('/localization/kinematic_state', Odometry, lambda m: receive('ego', m), queue_size=1)
    rospy.Subscriber('/perception/object_recognition/objects', PredictedObjects, lambda m: receive('objects', m), queue_size=1)
    rospy.Subscriber('/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
                     PathWithLaneId, lambda m: receive('bpp_path', m), queue_size=1)
    rospy.Subscriber('/planning/mission_planning/route', LaneletRoute, lambda m: receive('route', m), queue_size=1)
    rospy.Subscriber('/competition_route_requester/request_status', String,
                     lambda m: receive('route_status', m), queue_size=1)
    rospy.Subscriber('/planning/debug/behavior_path_planner/module_ownership', String,
                     lambda m: receive('ownership', m), queue_size=1)
    def receive_ack(message):
        try:receive('intent_ack_'+json.loads(message.data)['module'],message)
        except (ValueError,KeyError):pass
    rospy.Subscriber('/evaluation/overtaking_policy/intent_ack',String,receive_ack,queue_size=20)
    for module in modules:
        rospy.Subscriber('/planning/cooperate_status/' + module, CooperateStatusArray,
                         lambda m, key=module: receive(key, m), queue_size=1)
    services={}
    for direction in ('left','right'):
        endpoint=candidate_service_name(direction)
        rospy.wait_for_service(endpoint,timeout=5.0)
        require_candidate_service_type(endpoint,rosservice.get_service_type(endpoint))
        services[direction]=rospy.ServiceProxy(endpoint,CooperateCommands)
    publisher = rospy.Publisher('/evaluation/overtaking_policy/status', String, queue_size=1, latch=True)
    intent_publisher=rospy.Publisher('/evaluation/overtaking_policy/passing_intent',String,queue_size=1,latch=False)
    policy = RetryPassingPolicy()
    verified_lifecycle = None
    ownership_boot = None
    ownership_sequence = -1
    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    def uid(value):
        return bytes(value.uuid).hex()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        now = time.monotonic()
        snapshot = dict(valid=False, now=now, reason='waiting_inputs')
        try:
            if (map_path.stat().st_mtime_ns, map_path.stat().st_size) != map_stat:
                raise ValueError('map_file_changed_restart_required')
            for name in ['ego', 'objects', 'bpp_path'] + modules:
                if name not in latest or now-latest[name][1] > .4:
                    raise ValueError('stale_' + name)
            if 'route' not in latest:
                raise ValueError('missing_route')
            route = latest['route'][0]
            if uid(route.uuid) != uuid.UUID(cfg['route_b']['uuid']).hex:
                raise ValueError('not_verified_route_b')
            if 'route_status' not in latest:
                raise ValueError('missing_route_verification')
            route_status = json.loads(latest['route_status'][0].data)
            if not verified_route_status(route_status, route.header.stamp.to_sec()):
                raise ValueError('requester_route_not_verified')
            lifecycle = (route_status['generation'], route.header.stamp.to_nsec())
            if verified_lifecycle is None:
                verified_lifecycle = lifecycle
            elif lifecycle != verified_lifecycle:
                raise ValueError('route_lifecycle_changed_restart_policy_required')
            if 'ownership' not in latest or now-latest['ownership'][1] > .4:
                raise ValueError('authoritative_ownership_missing_or_stale')
            ownership_snapshot = json.loads(latest['ownership'][0].data)
            inactive = authoritative_inactive(ownership_snapshot, rospy.Time.now().to_sec(),
                uid(route.uuid), route.header.stamp.to_sec(), modules)
            if inactive is None:
                raise ValueError('authoritative_ownership_invalid_or_other_route')
            if not isinstance(ownership_snapshot.get('sequence'), int) or not ownership_snapshot.get('boot_id'):
                raise ValueError('ownership_missing_epoch')
            if ownership_boot is None:
                ownership_boot = ownership_snapshot['boot_id']
            if ownership_snapshot['boot_id'] != ownership_boot or ownership_snapshot['sequence'] < ownership_sequence:
                raise ValueError('ownership_epoch_changed_restart_policy_required')
            ownership_sequence = ownership_snapshot['sequence']
            preferred = [s.preferred_primitive.id for s in route.segments]
            if 201453 not in preferred or 201459 not in preferred:
                raise ValueError('route_missing_verified_highway')
            first, last = preferred.index(201459), preferred.index(201453)
            expected = cfg['route_b']['expected_path_lanelet_ids']
            if preferred[first:last+1] != expected[expected.index(201459):expected.index(201453)+1]:
                raise ValueError('highway_route_changed')
            corridor = set(p.id for segment in route.segments[first:last] for p in segment.primitives)
            successors = {lane: [] for lane in corridor}
            for source in corridor:
                sb = relations[source][1]
                for target in corridor:
                    if source == target:
                        continue
                    tb = relations[target][1]
                    if all(ways[sb[side]][-1] == ways[tb[side]][0] for side in ('left', 'right')):
                        successors[source].append(target)
            reference = [point for lane in preferred[first:last+1] for point in geometry.lanes[lane][0]]
            end = geometry.lanes[201453][0][0]
            end_s = project(reference, *end)[1]
            ego = latest['ego'][0]
            objects = latest['objects'][0]
            bpp_path = latest['bpp_path'][0]
            rostime = rospy.Time.now().to_sec()
            if any(m.header.frame_id != 'map' or not -.05 <= rostime-m.header.stamp.to_sec() <= .4 for m in [ego, objects, bpp_path]):
                raise ValueError('frame_or_header_age')
            if bpp_path.header.stamp < route.header.stamp:
                raise ValueError('lead_path_before_route')
            p, heading = ego.pose.pose.position, yaw(ego.pose.pose.orientation)
            if not all(math.isfinite(v) for v in (p.x, p.y, heading, ego.twist.twist.linear.x)):
                raise ValueError('nonfinite_ego')
            def locate(x, y, h):
                options = []
                for lane in corridor:
                    line, polygon = geometry.lanes[lane]
                    distance, arc, tangent = project(line, x, y)
                    if inside(polygon, x, y) and abs(math.atan2(math.sin(h-tangent), math.cos(h-tangent))) < math.pi/4:
                        options.append((distance, lane))
                return min(options)[1] if options else None
            lane = locate(p.x, p.y, heading)
            if lane is None:
                raise ValueError('outside_highway_lane_polygon')
            tags, bounds = relations[lane]
            speed_limit = float(tags.get('speed_limit', '0')) / 3.6
            if speed_limit < 80/3.6:
                raise ValueError('not_high_speed_road')
            def adjacent(direction):
                side, other = ('left', 'right') if direction == 'left' else ('right', 'left')
                for candidate in corridor:
                    if candidate == lane or bounds.get(side) != relations[candidate][1].get(other):
                        continue
                    tangent = project(geometry.lanes[candidate][0], p.x, p.y)[2]
                    if abs(math.atan2(math.sin(heading-tangent), math.cos(heading-tangent))) < math.pi/4:
                        return True
                return False
            def paired(direction):
                side,other=('left','right') if direction=='left' else ('right','left')
                sources=[];targets=[]
                for source in sorted(forward_lane_chain(lane,successors)):
                    source_bounds=relations[source][1]
                    choices=[target for target in corridor if target!=source and
                             source_bounds.get(side)==relations[target][1].get(other)]
                    if len(choices)!=1:continue
                    sources.append(source);targets.append(choices[0])
                return [sources,targets] if lane in sources and targets else None
            ego_s = project(reference, p.x, p.y)[1]
            lead_lanes = forward_lane_chain(lane, successors)
            path_objects, tracked = [], None
            object_ids = [uid(o.object_id) for o in objects.objects]
            if len(set(object_ids)) != len(object_ids):
                raise ValueError('ambiguous_duplicate_object_uuid')
            for obj in objects.objects:
                label = max(obj.classification, key=lambda c:c.probability).label if obj.classification else 0
                if label not in (1, 2, 3, 4):
                    continue
                pose = obj.kinematics.initial_pose_with_covariance.pose
                q = pose.position
                if not all(math.isfinite(v) for v in (q.x, q.y, yaw(pose.orientation),
                        obj.kinematics.initial_twist_with_covariance.twist.linear.x,
                        obj.shape.dimensions.x)) or obj.shape.dimensions.x <= 0:
                    continue
                obj_s = project(reference, q.x, q.y)[1]
                record = dict(uuid=uid(obj.object_id), gap=obj_s-ego_s,
                              speed=abs(obj.kinematics.initial_twist_with_covariance.twist.linear.x),
                              length=obj.shape.dimensions.x)
                if record['uuid'] == policy.lead:
                    tracked = record
                predictions = []
                for prediction in obj.kinematics.predicted_paths:
                    if not math.isfinite(prediction.confidence) or prediction.confidence <= 0:
                        continue
                    step = prediction.time_step.sec + prediction.time_step.nanosec*1e-9
                    if not math.isfinite(step) or step <= 0:
                        continue
                    predictions.extend((i*step, (v.position.x, v.position.y, yaw(v.orientation)))
                                       for i, v in enumerate(prediction.path) if i > 0)
                path_objects.append(dict(record, width=obj.shape.dimensions.y,
                    pose=(q.x, q.y, yaw(pose.orientation)), predictions=sorted(predictions)))
            leads = path_aligned_leads(
                [(v.point.pose.position.x, v.point.pose.position.y) for v in bpp_path.points],
                (p.x, p.y, heading), path_objects, ego_width)
            rtc = {}
            for direction in ('left', 'right'):
                entries = latest['external_request_lane_change_' + direction][0].statuses
                rtc[direction] = [dict(uuid=uid(r.uuid), state=r.state.type, safe=r.safe, auto_mode=r.auto_mode)
                                  for r in entries if -.05 <= rostime-(r.stamp.sec+r.stamp.nanosec*1e-9) <=
                                  (10.0 if r.state.type in (3, 4) and uid(r.uuid) == policy.candidate else .4)]
            ownership = {module: [dict(uuid=uid(r.uuid), state=r.state.type,
                stamp=r.stamp.sec+r.stamp.nanosec*1e-9) for r in latest[module][0].statuses]
                for module in modules}
            busy, ownership_evidence = ownership_busy(ownership, rostime, inactive)
            # Active manager evidence also blocks requests if RTC missed a sample.
            busy = busy or any(value['active'] for value in inactive.values())
            intent_ack=False;ack_reserve=150;mandatory_priority=False
            ack_key='intent_ack_external_request_lane_change_'+str(policy.direction)
            if ack_key in latest and now-latest[ack_key][1]<=.4 and policy.intent_id:
                ack=json.loads(latest[ack_key][0].data)
                ack_matches=(ack.get('intent_uuid')==policy.intent_id and ack.get('outbound_uuid')==policy.candidate
                    and ack.get('boot_id')==ownership_boot and ack.get('route_uuid')==uid(route.uuid)
                    and abs(float(ack.get('route_stamp',-1))-route.header.stamp.to_sec())<1e-6
                    and ack.get('module')=='external_request_lane_change_'+str(policy.direction)
                    and 0<=rostime-float(ack.get('stamp',-1))<=.4)
                intent_ack=ack_matches and ack.get('acknowledged') in (True,'true')
                if ack_matches:
                    ack_reserve=float(ack.get('reserve',150))
                    if not math.isfinite(ack_reserve) or ack_reserve<0:
                        raise ValueError('invalid_ack_reserve')
                    mandatory_priority=ack.get('reason')=='mandatory_route_priority'
            snapshot.update(valid=True, rtc=rtc, busy=busy, remaining=end_s-ego_s,
                intent_ack=intent_ack,ack_reserve=ack_reserve,mandatory_priority=mandatory_priority,
                ros_now=rostime,ownership_sequence=ownership_snapshot['sequence'],
                rtc_auto_owners=[dict(module=module,uuid=uid(r.uuid),state=r.state.type)
                    for module in modules if module.startswith('external_request_')
                    for r in latest[module][0].statuses if r.auto_mode and r.state.type not in (3,4)],
                intent_route=(uid(route.uuid),route.header.stamp.to_sec()),intent_boot=ownership_boot,
                paired_chains={d:paired(d) for d in ('left','right')},
                ownership_evidence=ownership_evidence, lead_lane_chain=sorted(lead_lanes),
                lead_path_stamp=bpp_path.header.stamp.to_sec(), lead_path_age=rostime-bpp_path.header.stamp.to_sec(),
                authoritative_owners=inactive,
                left_allowed=adjacent('left'), right_allowed=adjacent('right'),
                ego_speed=abs(ego.twist.twist.linear.x), desired_speed=min(speed_limit, 120/3.6),
                lead=min(leads, key=lambda o:o['gap']) if leads else None,
                tracked_lead_present=tracked is not None,
                fully_passed=bool(tracked and tracked['gap'] < -(tracked['length']/2+4.635+8)), lane=lane)
        except (ValueError, KeyError, IndexError) as exc:
            snapshot.update(valid=False, reason=str(exc))
        request = policy.step(snapshot)
        intent=policy.intent(snapshot,rospy.Time.now().to_sec(),map_hash)
        if intent:intent_publisher.publish(String(data=json.dumps(intent)))
        result = None
        if request:
            try:
                endpoint=candidate_service_name(request)
                require_candidate_service_type(endpoint,rosservice.get_service_type(endpoint))
                # Only the matching opt-in server accepts this as a SAFE AUTO
                # lease. It retains DEACTIVATE command state, checks sole fresh
                # safe WAITING UUID atomically, and never sets manual override.
                command=CooperateCommand()
                command.uuid.uuid=list(bytes.fromhex(policy.candidate))
                command.module.type=(Module.EXT_REQUEST_LANE_CHANGE_LEFT if request=='left'
                                     else Module.EXT_REQUEST_LANE_CHANGE_RIGHT)
                command.command.type=Command.ACTIVATE
                rpc=CooperateCommandsRequest();stamp=rospy.Time.now()
                rpc.stamp.sec=stamp.secs;rpc.stamp.nanosec=stamp.nsecs;rpc.commands=[command]
                response=services[request](rpc)
                result=atomic_response_outcome(policy.candidate,command.module.type,
                    [dict(uuid=uid(r.uuid),module=r.module.type,success=r.success) for r in response.responses])
                policy.record_rpc_result(snapshot,result)
            except (rospy.ServiceException,rosservice.ROSServiceException,RuntimeError) as exc:
                result = str(exc)
                policy.record_rpc_result(snapshot,None)
        publisher.publish(String(data=json.dumps(dict(state=policy.state, reason=policy.reason,
            candidate=policy.candidate, lead=policy.lead, requested=request, service_result=result,
            attempt_count=policy.attempt_count,outcome_reason=policy.outcome_reason,
            retired_candidates=sorted(policy.retired),retry_after=policy.retry_after,
            evidence=snapshot), sort_keys=True)))
        rate.sleep()


if __name__ == '__main__':
    main()
