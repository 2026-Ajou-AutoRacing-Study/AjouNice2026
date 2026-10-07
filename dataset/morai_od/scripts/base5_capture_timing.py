"""Observation-only monotonic stage accounting; never alters capture gates."""
import time
import math


def post_load_topics_ready(state, actors, load_time, now):
    """Fresh post-load receive witnesses; AI annotation-only exception stays scoped."""
    if any(state[k]<=load_time+.5 or now-state[k]>.8
           for k in ('ego_t','object_t','collision_t')):
        return False
    expected={a['unique_id'] for a in actors if a['source_list']!='aiPedstrianList'}
    objects=state.get('objects') or {}
    seen={a['unique_id'] for k in ('npc_list','pedestrian_list','obstacle_list')
          for a in objects.get(k,[])}
    return expected<=seen


def deferred_static_secondary(state,actors,load_time,now,allow_missing_primary=False):
    """Fresh transport plus present primary; missing static secondary is review,
    not a transport outage. Legacy callers do not defer a missing primary.
    Explicit lightweight all-static captures may diagnose a missing primary
    using the original annotation; its absence remains a quality review.
    Never defer an animated actor or stale transport.
    The subsequent capture still requires Park/contact/pose and presence audits.
    """
    if not actors or not post_load_topics_ready(state,[],load_time,now):return []
    seen={a['unique_id'] for k in ('npc_list','pedestrian_list','obstacle_list')
          for a in (state.get('objects') or {}).get(k,[])}
    if actors[0]['source_list']!='aiPedstrianList' and actors[0]['unique_id'] not in seen:
        if not allow_missing_primary or any(a.get('motion') not in ('static','standing') for a in actors):return []
    missing=[a for a in actors if a['source_list']!='aiPedstrianList' and a['unique_id'] not in seen]
    if not missing or any(a.get('motion') not in ('static','standing') for a in missing):return []
    return [a['unique_id'] for a in missing]


def preflight_stationary(samples, now, window=.30):
    """Require recent finite, near-zero-speed and stable-pose receive evidence."""
    rows=[r for r in samples if now-window-.10 <= r['t'] <= now]
    keys=('t','x','y','z','vx','vy')
    if len(rows)<3 or any(not all(math.isfinite(r[k]) for k in keys) for r in rows):
        return False
    if now-rows[-1]['t']>.10 or rows[-1]['t']-rows[0]['t']<window:
        return False
    return (all(math.hypot(r['vx'],r['vy'])<.02 for r in rows)
            and math.hypot(max(r['x'] for r in rows)-min(r['x'] for r in rows),
                           max(r['y'] for r in rows)-min(r['y'] for r in rows))<.02
            and max(r['z'] for r in rows)-min(r['z'] for r in rows)<.02)


def stationary_witness(samples,now,not_before,window=.30):
    """Post-Park proof includes vertical velocity; never trust gear ACK alone."""
    rows=[r for r in samples if max(not_before,now-window-.10)<=r['t']<=now]
    finite=bool(rows) and all(math.isfinite(r.get('vz',float('nan'))) for r in rows)
    stable=preflight_stationary(rows,now,window) and finite and all(abs(r['vz'])<.02 for r in rows)
    def span(key):
        values=[r[key] for r in rows if math.isfinite(r[key])]
        return max(values)-min(values) if values else None
    xy=(math.hypot(span('x'),span('y')) if rows and span('x') is not None and span('y') is not None else None)
    return dict(policy='post-park-stationary-v114',verified=stable,not_before_monotonic=not_before,
        observed_at_monotonic=now,window_sec=window,sample_count=len(rows),
        receive_span_sec=rows[-1]['t']-rows[0]['t'] if rows else None,
        xy_span_m=xy,z_span_m=span('z'),
        maximum_xy_speed_mps=max((math.hypot(r['vx'],r['vy']) for r in rows),default=None),
        maximum_abs_vz_mps=max((abs(r.get('vz',float('inf'))) for r in rows),default=None),
        thresholds=dict(xy_span_m=.02,z_span_m=.02,xy_speed_mps=.02,abs_vz_mps=.02),samples=rows)


class StageTiming:
    def __init__(self, clock=time.perf_counter):
        self.clock = clock
        self.started = self.last = clock()
        self.intervals = []

    def mark(self, stage):
        now = self.clock()
        self.intervals.append(dict(stage=stage, seconds=now-self.last))
        self.last = now

    def snapshot(self):
        totals = {}
        for interval in self.intervals:
            stage = interval['stage']
            totals[stage] = totals.get(stage, 0.0)+interval['seconds']
        return dict(schema=1, total_sec=self.last-self.started,
                    stages_sec=totals, intervals=list(self.intervals),
                    limitation='Wall-clock stages include waits and CPU work; not simulator internal timing. Interpreter startup and final timing-file write excluded.')
