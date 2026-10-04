"""Offline supplementary GT overlap accounting, never a driving decision."""
import math


class ObjectClockOverlaps:
    def __init__(self, intervention_stamp=None):
        self.intervention_stamp = intervention_stamp
        self.by_phase = {p: {} for p in (
            'before_intervention', 'after_intervention', 'unknown_boundary')}
        self.all = {}

    def record(self, object_id, stamp, elapsed, ego_age, area):
        # Same freshness bound as the existing evaluator. No extrapolation,
        # retimestamping, duplicate-frame suppression, or safety-gate changes.
        if not all(math.isfinite(x) for x in (stamp, elapsed, ego_age, area)):
            return False
        if not 0 <= ego_age < .2 or area < 0:
            return False
        phase = ('unknown_boundary' if self.intervention_stamp is None else
                 'before_intervention' if stamp < self.intervention_stamp else
                 'after_intervention')
        for bucket in (self.all, self.by_phase[phase]):
            item = bucket.setdefault(object_id, dict(
                first_sec=elapsed, samples=0, maximum_area_m2=0.,
                maximum_ego_age_sec=0.))
            item.update(last_sec=elapsed, samples=item['samples']+1,
                        maximum_area_m2=max(item['maximum_area_m2'], area),
                        maximum_ego_age_sec=max(item['maximum_ego_age_sec'], ego_age))
        return True
