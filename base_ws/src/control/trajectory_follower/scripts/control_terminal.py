#!/usr/bin/env python3
"""Read-only CTRL terminal. No Ego input, control decisions or command publisher."""
from datetime import datetime
import math
import os
import re
import sys
import threading
import time


class Terminal:
    def __init__(self, emit, timeout=0.5):
        self.emit = emit
        self.timeout = timeout
        self.samples = {}
        self.status = None
        self.fatal = None
        self.last_key = None
        self.last_output = -math.inf

    def sample(self, name, value, now):
        self.samples[name] = (value, now)

    def state(self, level, reason, steering, now):
        self.status = (level, reason, steering, now)

    def value(self, name, now):
        value, received = self.samples.get(name, (math.nan, -math.inf))
        return value if now - received <= self.timeout and math.isfinite(value) else None

    def tick(self, now, stamp):
        level, detail = 'WARN', 'WAIT controller'
        category = 'waiting'
        if self.fatal:
            level, detail, category = 'ERROR', 'FATAL ' + self.fatal, 'fatal'
        elif self.status is not None:
            state, reason, steering, received = self.status
            if now - received > self.timeout:
                level, detail, category = 'ERROR', 'ERROR controller timeout', 'timeout'
            elif state in ('ERROR', 'WAIT'):
                level = 'ERROR' if state == 'ERROR' else 'WARN'
                detail = state + ' ' + reason
                category = re.sub(r'[-+]?\d+(?:\.\d+)?', '#', detail)
            else:
                vx, ay = self.value('vx', now), self.value('ay', now)
                steer = math.degrees(steering)
                valid_steer = math.isfinite(steer)
                # Small tolerance handles conversion roundoff at exactly 40 deg.
                saturated = valid_steer and abs(steer) >= 40.0 - 1e-6
                level = ('WARN' if state == 'WARN' or vx is None or ay is None or saturated
                         else 'OK')
                if not valid_steer:
                    level = 'ERROR'
                speed_text = '%6.2f' % vx if vx is not None else '%6s' % '--'
                ay_text = '%5.2f' % ay if ay is not None else '%5s' % '--'
                steer_text = '%+5.1f' % steer if valid_steer else '%5s' % '--'
                detail = '%skph %smps2 %s°' % (speed_text, ay_text, steer_text)
                category = 'numeric'
        key = (level, category)
        period = 1.0 if level == 'OK' else 0.2
        # Every severity transition, including the FIRST recovered sample, emits now.
        if key != self.last_key or now - self.last_output >= period - 1e-9:
            self.emit('%s CTRL %s' % (stamp, detail), level)
            self.last_key, self.last_output = key, now


def main():
    import rospy
    from morai_msgs.msg import EgoVehicleStatus
    from rosgraph_msgs.msg import Log
    from sensor_msgs.msg import Imu
    from trajectory_follower.msg import TerminalState

    rospy.init_node('control_terminal')
    lock = threading.RLock()
    color = sys.stdout.isatty() and 'NO_COLOR' not in os.environ and os.environ.get('TERM') != 'dumb'
    output_failed = False

    def emit(line, level):
        nonlocal output_failed
        if output_failed:
            return
        if color:
            line = '\033[%sm%s\033[0m' % ({'OK': '92', 'WARN': '93', 'ERROR': '91'}[level], line)
        try:
            print(line, flush=True)
        except (OSError, ValueError):
            # Display loss must never stop the driving nodes.
            output_failed = True

    terminal = Terminal(emit, rospy.get_param('~display_timeout_s', 0.5))

    def tick():
        terminal.tick(time.monotonic(), datetime.now().strftime('%S.%f')[:4])

    def competition(message):
        with lock:
            # The Competition ROS bridge already converts native km/h to m/s.
            terminal.sample('vx', message.velocity.x * 3.6, time.monotonic())
            tick()

    def imu(message):
        with lock:
            terminal.sample('ay', message.linear_acceleration.y, time.monotonic())
            tick()

    def state(message):
        with lock:
            terminal.state(message.level, message.reason, message.steering_rad, time.monotonic())
            tick()

    def logs(message):
        if message.name in ('/purepursuit_autoware', '/longi_pi_autoware') and message.level == Log.FATAL:
            with lock:
                terminal.fatal = re.sub(r'\x1b\[[0-9;]*m', '', message.msg)
                tick()

    subscribers = [
        rospy.Subscriber('/Competition_topic', EgoVehicleStatus, competition, queue_size=1),
        rospy.Subscriber('/sensor/imu', Imu, imu, queue_size=1),
        rospy.Subscriber('/control/terminal_state', TerminalState, state, queue_size=1),
        rospy.Subscriber('/rosout', Log, logs, queue_size=10),
    ]
    # Wall time keeps the display/watchdog alive when simulation ROS time pauses.
    while not rospy.is_shutdown():
        with lock:
            tick()
        time.sleep(0.01)
    for subscriber in subscribers:
        subscriber.unregister()


if __name__ == '__main__':
    main()
