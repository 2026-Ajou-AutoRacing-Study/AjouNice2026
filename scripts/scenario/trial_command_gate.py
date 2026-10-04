"""Trial-only command ownership; stopping never releases a latched brake."""
import copy
import math
import threading
import time


class TrialCommandGate:
    def __init__(self, publish, empty_command):
        self.publish = publish
        self.empty_command = empty_command
        self.lock = threading.RLock()
        self.last = None
        self.last_valid_steering = 0.0
        self.stopped = False
        self.previous_brake_time = None
        self.metrics = dict(brake_messages=0, max_brake_publish_gap_sec=0.0,
                            post_latch_accel_nonzero=0)

    def _brake(self):
        command = copy.deepcopy(self.last) if self.last is not None else self.empty_command()
        command.longlCmdType = 1
        command.accel = 0.0
        command.brake = 1.0
        command.velocity = 0.0
        command.acceleration = 0.0
        command.steering = self.last_valid_steering
        return command

    def _publish_brake(self):
        command = self._brake()
        now = time.monotonic()
        if self.previous_brake_time is not None:
            self.metrics['max_brake_publish_gap_sec'] = max(
                self.metrics['max_brake_publish_gap_sec'], now-self.previous_brake_time)
        self.previous_brake_time = now
        self.metrics['brake_messages'] += 1
        self.metrics['post_latch_accel_nonzero'] += int(command.accel != 0.)
        self.publish(command)

    def receive(self, command):
        with self.lock:
            self.last = copy.deepcopy(command)
            if math.isfinite(command.steering):
                self.last_valid_steering = command.steering
            if self.stopped:
                self._publish_brake()
            else:
                self.publish(command)

    def stop(self):
        with self.lock:
            self.stopped = True
            self._publish_brake()

    def tick(self):
        with self.lock:
            if self.stopped:
                self._publish_brake()
