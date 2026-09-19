"""ROS-independent timing rules used by the competition monitor."""


def is_speeding(speed_kph, limit_kph, in_exception=False):
    """The agreed monitor contract treats exactly 60.0 kph as a violation."""
    return not in_exception and float(speed_kph) >= float(limit_kph)


class PeriodicPenalty:
    """Track one continuous condition with optional immediate first penalty."""

    def __init__(self, interval_sec, penalty_sec, immediate=False):
        self.interval_sec = float(interval_sec)
        self.penalty_sec = int(penalty_sec)
        self.immediate = bool(immediate)
        self.started_at = None
        self.issued = 0

    @property
    def active(self):
        return self.started_at is not None

    def begin(self, now):
        if self.active:
            return []
        self.started_at = float(now)
        self.issued = 0
        if self.immediate:
            self.issued = 1
            return [(0.0, self.penalty_sec)]
        return []

    def update(self, now):
        if not self.active:
            return []
        elapsed = max(0.0, float(now) - self.started_at)
        due = int(elapsed // self.interval_sec) + (1 if self.immediate else 0)
        events = []
        while self.issued < due:
            threshold = (
                0.0 if self.immediate and self.issued == 0
                else self.interval_sec * (self.issued if self.immediate else self.issued + 1)
            )
            self.issued += 1
            events.append((threshold, self.penalty_sec))
        return events

    def end(self, now):
        if not self.active:
            return None
        result = {
            "started_at": self.started_at,
            "ended_at": float(now),
            "duration": max(0.0, float(now) - self.started_at),
            "penalties": self.issued,
            "penalty_seconds": self.issued * self.penalty_sec,
        }
        self.started_at = None
        self.issued = 0
        return result


class OrderedCheckpoints:
    def __init__(self, count):
        self.count = int(count)
        self.passed = 0

    @property
    def next_number(self):
        return self.passed + 1 if self.passed < self.count else None

    def observe(self, checkpoint_number, inside):
        if inside and checkpoint_number == self.next_number:
            self.passed += 1
            return True
        return False

    def reset(self):
        self.passed = 0
