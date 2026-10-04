"""Bounded collision/GT association. Caller serializes access with its state lock."""
import math


class PendingCollisionReview:
    def __init__(self, wait_sec=0.1):
        if not 0 < wait_sec <= 0.1:
            raise ValueError('collision association wait must be <= 0.1 seconds')
        self.wait_sec = wait_sec
        self.pending = {}

    def report(self, object_id, xy, now, context):
        # Repeated collision packets must not postpone the first deadline.
        self.pending.setdefault(object_id, dict(object_id=object_id, xy=xy,
            first=now, deadline=now+self.wait_sec, context=context))

    def resolve(self, now, objects, gt_time, gap):
        """Objects: id -> {xy, speed, valid, ...}; gap(event, object) -> meters/None."""
        outcomes = []
        for object_id, event in list(self.pending.items()):
            if now >= event['deadline']:
                outcomes.append(dict(event=event, review=True, reason='collision_gt_association_timeout'))
                del self.pending[object_id]
                continue
            obj = objects.get(object_id)
            if obj is None or not obj.get('valid') or not 0 <= now-gt_time <= .2:
                continue
            dt = abs(gt_time-event['first'])
            speed = obj['speed']
            if not math.isfinite(speed) or speed < 0 or speed > 60:
                continue
            # Packet pose jitter plus motion/acceleration over association time.
            tolerance = .5 + speed*dt + .5*10*dt*dt
            distance = math.hypot(obj['xy'][0]-event['xy'][0], obj['xy'][1]-event['xy'][1])
            if not math.isfinite(distance) or distance > tolerance:
                continue
            separation = gap(event, obj)
            if separation is None or not math.isfinite(separation):
                continue
            outcomes.append(dict(event=event, review=separation <= .5,
                reason='collision_gt_near' if separation <= .5 else 'distant_npc_collision_rejected',
                gap_m=separation, association_distance_m=distance, association_wait_sec=now-event['first']))
            del self.pending[object_id]
        return outcomes
