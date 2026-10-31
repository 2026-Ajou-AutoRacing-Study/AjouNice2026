#!/usr/bin/python3
"""DEVELOPMENT STRESS ONLY: one bounded traffic-light yaw-covariance pulse.

Reads real localization odometry and traffic-light runtime observations. The
only publisher is an evaluation odometry topic; no signal, TF, native API or
vehicle-control messages are generated. Header, pose, twist and twist covariance
are copied unchanged; only pose.covariance[35] may change. This is not production
configuration and must be paired with the owner's controlled local drive test.
"""
import argparse
import copy
import json
import math
import threading
import time


INPUT_TOPIC = '/localization/kinematic_state'
OUTPUT_TOPIC = '/evaluation/traffic_light_policy/covariance_odometry'
DEBUG_TOPIC = '/traffic_light_node/debug_runtime'
TARGET_GROUP = 201351
FRESH_SEC = .5


def bounded(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(name + ' must be numeric')
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError('{} must be within {}..{}'.format(name, low, high))
    return float(value)


def stamp_ns(message):
    stamp = message.header.stamp
    return int(stamp.secs) * 1000000000 + int(stamp.nsecs)


def fresh(source_ns, receipt, now, ros_now_ns):
    return (type(source_ns) is int and source_ns > 0 and
            0 <= now - receipt <= FRESH_SEC and
            0 <= (ros_now_ns - source_ns) / 1e9 <= FRESH_SEC)


def current_observed_color(report, now_ns):
    """Accept actual current A-group lamps, not held/synthetic permission."""
    if not isinstance(report, dict):
        return None
    source = report.get('source_stamp_ns')
    frame = report.get('frame_decision')
    if not isinstance(frame, dict) or type(frame.get('group_id')) is not int:
        return None
    if frame['group_id'] != TARGET_GROUP:
        return None
    if not fresh(source, 0., 0., now_ns):
        return None
    timing = frame.get('observation_timing')
    if (frame.get('selection_status') != 'matched' or
            frame.get('classifier_known') is not True or
            not isinstance(timing, dict) or timing.get('output_is_observed') is not True or
            timing.get('adopted_observation_source_stamp_ns') != source or
            timing.get('output_source_stamp_ns') != source):
        return None
    observed = frame.get('observed_tuples')
    final = frame.get('final_tuples')
    for color in ('red', 'green'):
        expected = [[color, 'circle', None]]
        if observed == expected and final == expected:
            return color
    return None


def covariance_copy(message, yaw_variance=None):
    """Pure copy: deliberately never edits stamps or any vehicle state."""
    result = copy.deepcopy(message)
    if yaw_variance is not None:
        if len(result.pose.covariance) != 36:
            raise ValueError('Odometry pose covariance must contain 36 entries')
        covariance = list(result.pose.covariance)
        covariance[35] = yaw_variance
        result.pose.covariance = covariance
    return result


class StressRelay:
    """ROS-free, dependency-injected one-shot gate; never invents input data."""
    def __init__(self, start_monotonic, duration_sec=.2, yaw_stddev_deg=3.2,
                 max_seconds=180., emit=None):
        self.duration_sec = bounded(duration_sec, .1, .3, 'duration_sec')
        self.yaw_stddev_deg = bounded(yaw_stddev_deg, 1., 5., 'yaw_stddev_deg')
        self.max_seconds = bounded(max_seconds, .001, 180., 'max_seconds')
        self.yaw_variance = math.radians(self.yaw_stddev_deg) ** 2
        self.start = start_monotonic
        self.deadline = start_monotonic + self.max_seconds
        self.emit = emit if emit is not None else lambda event, **fields: None
        self.signal = None
        self.odom = None
        self.red_armed = False
        self.trigger_monotonic = None
        self.trigger_stamp_ns = None
        self.mutated_count = 0
        self.relayed_count = 0
        self.pulse_finished = False
        self.stopped = False

    def expired(self, now):
        return self.stopped or now >= self.deadline

    def stop(self, reason):
        if not self.stopped:
            self.stopped = True
            self.emit('stress_shutdown', reason=reason, triggered=self.trigger_monotonic is not None,
                      trigger_stamp_ns=self.trigger_stamp_ns, mutated_count=self.mutated_count,
                      relayed_count=self.relayed_count)

    def observe_debug(self, report, now, ros_now_ns):
        if self.expired(now):
            return
        color = current_observed_color(report, ros_now_ns)
        self.signal = (color, now, report.get('source_stamp_ns')) if color else None
        frame = report.get('frame_decision') if isinstance(report, dict) else None
        if isinstance(frame, dict) and frame.get('group_id') != TARGET_GROUP:
            self.red_armed = False
        self._arm_red(now, ros_now_ns)

    def _arm_red(self, now, ros_now_ns):
        if self.red_armed or self.trigger_monotonic is not None or not self.signal or not self.odom:
            return
        color, signal_receipt, signal_stamp = self.signal
        speed, odom_receipt, odom_stamp = self.odom
        if (color == 'red' and math.isfinite(speed) and speed < .1 and
                fresh(signal_stamp, signal_receipt, now, ros_now_ns) and
                fresh(odom_stamp, odom_receipt, now, ros_now_ns)):
            self.red_armed = True
            self.emit('stress_armed_observed_red_while_stopped', group_id=TARGET_GROUP,
                      signal_source_stamp_ns=signal_stamp, odometry_stamp_ns=odom_stamp,
                      speed_mps=speed)

    def relay(self, message, now, ros_now_ns):
        """Return one copy per received odometry; None after the hard deadline."""
        if self.expired(now):
            return None
        velocity = message.twist.twist.linear
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        source = stamp_ns(message)
        self.odom = (speed, now, source)
        self._arm_red(now, ros_now_ns)
        if self.trigger_monotonic is None and self.red_armed and self.signal:
            color, receipt, signal_stamp = self.signal
            if (color == 'green' and .3 <= speed <= 2. and
                    fresh(signal_stamp, receipt, now, ros_now_ns) and
                    fresh(source, now, now, ros_now_ns)):
                self.trigger_monotonic = now
                self.trigger_stamp_ns = source
                self.emit('stress_trigger', group_id=TARGET_GROUP, trigger_stamp_ns=source,
                          trigger_monotonic=now, signal_source_stamp_ns=signal_stamp,
                          speed_mps=speed, duration_sec=self.duration_sec,
                          original_yaw_covariance=message.pose.covariance[35],
                          new_yaw_covariance=self.yaw_variance, mutated_count=1)
        active = (self.trigger_monotonic is not None and
                  self.trigger_monotonic <= now < self.trigger_monotonic + self.duration_sec)
        if active:
            self.mutated_count += 1
        elif self.trigger_monotonic is not None and not self.pulse_finished:
            self.pulse_finished = True
            self.emit('stress_pulse_finished', trigger_stamp_ns=self.trigger_stamp_ns,
                      mutated_count=self.mutated_count, elapsed_sec=now-self.trigger_monotonic)
        self.relayed_count += 1
        return covariance_copy(message, self.yaw_variance if active else None)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--duration-sec', type=float, default=.2, help='Stress pulse, .1.. .3 seconds')
    result.add_argument('--yaw-stddev-deg', type=float, default=3.2, help='Stress yaw sigma, 1..5 degrees')
    result.add_argument('--max-seconds', type=float, default=180., help='Wall-clock lifetime, at most 180 seconds')
    return result


def run(args):
    # Validate before importing/initializing ROS; pure tests import this module safely.
    def emit(event, **fields):
        print(json.dumps(dict(event=event, development_stress_only=True,
                              monotonic=time.monotonic(), **fields), allow_nan=False), flush=True)

    relay = StressRelay(time.monotonic(), args.duration_sec, args.yaw_stddev_deg,
                        args.max_seconds, emit)
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    rospy.init_node('evaluation_stress_signal_covariance_relay', anonymous=False)
    for topic in (INPUT_TOPIC, OUTPUT_TOPIC, DEBUG_TOPIC):
        if rospy.resolve_name(topic) != topic:
            raise ValueError('Stress topics are fixed; remapping is not permitted: ' + topic)
    lock = threading.Lock()
    done = threading.Event()
    publisher = rospy.Publisher(OUTPUT_TOPIC, Odometry, queue_size=10, latch=False)
    subscribers = []

    def odometry_callback(message):
        with lock:
            if done.is_set():
                return
            try:
                result = relay.relay(message, time.monotonic(), rospy.Time.now().to_nsec())
                if result is not None:
                    publisher.publish(result)
            except Exception as error:
                emit('stress_callback_error', detail=str(error))
                done.set()

    def debug_callback(message):
        with lock:
            if done.is_set():
                return
            try:
                report = json.loads(message.data)
            except (ValueError, TypeError):
                report = None
            relay.observe_debug(report, time.monotonic(), rospy.Time.now().to_nsec())

    rospy.on_shutdown(done.set)
    emit('stress_relay_started', input_topic=INPUT_TOPIC, output_topic=OUTPUT_TOPIC,
         debug_topic=DEBUG_TOPIC, target_group_id=TARGET_GROUP,
         max_seconds=relay.max_seconds, duration_sec=relay.duration_sec,
         yaw_stddev_deg=relay.yaw_stddev_deg, new_yaw_covariance=relay.yaw_variance)
    try:
        subscribers.append(rospy.Subscriber(INPUT_TOPIC, Odometry, odometry_callback, queue_size=10))
        subscribers.append(rospy.Subscriber(DEBUG_TOPIC, String, debug_callback, queue_size=1))
        while not rospy.is_shutdown() and not done.wait(.05):
            if relay.expired(time.monotonic()):
                break
    finally:
        done.set()
        with lock:
            relay.stop('max_seconds' if time.monotonic() >= relay.deadline else 'owner_or_callback_shutdown')
        for subscriber in subscribers:
            subscriber.unregister()
        publisher.unregister()
        if not rospy.is_shutdown():
            rospy.signal_shutdown('development covariance stress relay stopped')


if __name__ == '__main__':
    run(parser().parse_args())
