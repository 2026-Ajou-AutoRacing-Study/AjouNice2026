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
