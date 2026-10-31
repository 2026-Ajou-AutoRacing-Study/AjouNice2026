#!/usr/bin/env python3
"""Passive bounded ROS timing/system observer. Never publishes or stores payloads."""
import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
TOPICS = {
    'trajectory': '/planning/scenario_planning/trajectory',
    'trajectory_legacy': '/planning/trajectory',
    'localization': '/localization/kinematic_state',
    'control_localization': '/localization/control_kinematic_state',
    'control_command': '/ctrl_cmd',
    'traffic': '/perception/traffic_light_recognition/traffic_signals',
    'gps': '/sensor/gps', 'imu': '/sensor/imu',
    'front_image': '/sensor/camera/front/compressed',
}
PROCESS_LABELS = ('traffic_light_node', 'behavior_velocity_planner',
                  'motion_velocity_planner', 'purepursuit', 'localization',
                  'rviz', 'Simulator.x86_64', 'roscore')


def quantiles(values):
    data = sorted(value for value in values if math.isfinite(value))
    if not data:
        return None
    def at(fraction):
        index = (len(data) - 1) * fraction
        lower = int(index)
        return data[lower] + (data[min(lower + 1, len(data) - 1)] - data[lower]) * (index - lower)
    return dict(n=len(data), mean=sum(data) / len(data), min=data[0],
                p50=at(.5), p90=at(.9), p95=at(.95), p99=at(.99), max=data[-1])


def source_seconds(message):
    stamp = getattr(getattr(message, 'header', None), 'stamp', None)
    if stamp is None:
        stamp = getattr(message, 'stamp', None)
    if stamp is None:
        return None, 'header_absent'
    seconds = getattr(stamp, 'secs', getattr(stamp, 'sec', None))
    nanoseconds = getattr(stamp, 'nsecs', getattr(stamp, 'nanosec', None))
    if (not isinstance(seconds, int) or isinstance(seconds, bool) or seconds < 0
            or not isinstance(nanoseconds, int) or isinstance(nanoseconds, bool)
            or not 0 <= nanoseconds < 10**9 or seconds + nanoseconds == 0):
        return None, 'invalid_source_stamp'
    return seconds + nanoseconds * 1e-9, None


class TopicStats:
    def __init__(self, expect_stamp=True, max_samples=6000):
        self.expect_stamp = expect_stamp
        self.counts = Counter()
        self.intervals = deque(maxlen=max_samples)
        self.ages = deque(maxlen=max_samples)
        self.last_mono = None
        self.first_mono = None

    def observe(self, message, receipt_ros, receipt_mono):
        self.counts['messages'] += 1
        if self.first_mono is None:
            self.first_mono = receipt_mono
        if self.last_mono is not None:
            if receipt_mono >= self.last_mono:
                self.intervals.append((receipt_mono - self.last_mono) * 1000.)
            else:
                self.counts['receipt_order_inversion'] += 1
        self.last_mono = (receipt_mono if self.last_mono is None
                          else max(receipt_mono, self.last_mono))
        if not self.expect_stamp:
            self.counts['unstamped_by_contract'] += 1
            return
        source, error = source_seconds(message)
        if error:
            self.counts[error] += 1
        elif not math.isfinite(receipt_ros) or receipt_ros <= 0:
            self.counts['invalid_receipt_ros_time'] += 1
        elif source > receipt_ros:
            self.counts['future_source_stamp'] += 1
        else:
            self.ages.append((receipt_ros - source) * 1000.)
            self.counts['valid_age_samples'] += 1

    def snapshot(self, now):
        span = None if self.first_mono is None else self.last_mono - self.first_mono
        return dict(counts=dict(self.counts), age_ms=quantiles(self.ages),
                    receipt_interval_ms=quantiles(self.intervals),
                    observed_hz=((self.counts['messages'] - 1) / span if span else None),
                    last_receipt_age_sec=(now - self.last_mono if self.last_mono is not None else None),
                    age_kind='source_stamp_to_callback_ros_time' if self.expect_stamp
                    else 'unavailable_no_header_not_latency',
                    rolling_max_samples=self.ages.maxlen)


class SystemSampler:
    def __init__(self):
        self.previous_cpu = None
        self.previous_process = {}
        self.processes = {}
        self.next_scan = 0.
        self.ticks = os.sysconf('SC_CLK_TCK')

    def sample(self, now, output):
        result = {}
        try:
            lines = Path('/proc/stat').read_text().splitlines()
            values = [int(value) for value in lines[0].split()[1:9]]
            total, idle = sum(values), values[3] + values[4]
            if self.previous_cpu:
                dt, di = total - self.previous_cpu[0], idle - self.previous_cpu[1]
                result['cpu_busy_percent'] = 100. * (dt - di) / dt if dt else None
            self.previous_cpu = total, idle
            result['runqueue_processes'] = int(next(line.split()[1] for line in lines
                                                    if line.startswith('procs_running ')))
            info = dict((line.split(':')[0], int(line.split()[1]))
                        for line in Path('/proc/meminfo').read_text().splitlines())
            result.update(mem_available_kib=info.get('MemAvailable'),
                          swap_used_kib=info.get('SwapTotal', 0) - info.get('SwapFree', 0),
                          disk_available_bytes=os.statvfs(output).f_bavail * os.statvfs(output).f_frsize)
        except (OSError, ValueError, StopIteration, IndexError) as error:
            result['system_read_error'] = type(error).__name__
        if now >= self.next_scan:
            self.processes = {}
            for entry in Path('/proc').iterdir():
                if not entry.name.isdigit():
                    continue
                try:
                    # Match internally; command lines/arguments are never emitted.
                    command = (entry / 'cmdline').read_bytes()[:16384].decode(errors='replace')
                    labels = [label for label in PROCESS_LABELS if label in command]
                    if labels:
                        self.processes[int(entry.name)] = labels
                except OSError:
                    continue
            self.next_scan = now + 5.
        processes = []
        for pid, labels in self.processes.items():
            try:
                fields = Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')', 1)[1].split()
                ticks, start = int(fields[11]) + int(fields[12]), int(fields[19])
                old = self.previous_process.get(pid)
                cpu = (100. * (ticks - old[1]) / self.ticks / (now - old[0])
                       if old and old[2] == start and now > old[0] else None)
                self.previous_process[pid] = now, ticks, start
                processes.append(dict(pid=pid, labels=labels, cpu_one_core_percent=cpu))
            except (OSError, ValueError, IndexError):
                continue
        result['processes'] = processes
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-sec', type=float, default=600.)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--front-topic', default=TOPICS['front_image'])
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration_sec) or not 0 < args.duration_sec <= 1800:
        parser.error('--duration-sec must be in (0, 1800]')
    base = (ROOT / 'artifacts' / 'evaluation').resolve()
    output = (args.output_dir or base / ('traffic_runtime_' +
              datetime.now().strftime('%Y%m%d_%H%M%S_%f'))).resolve()
    if base not in output.parents:
        parser.error('--output-dir must be a fresh directory under artifacts/evaluation')
    output.mkdir(parents=True, exist_ok=False)
    # ROS is optional on import: pure statistic tests require neither ROS nor a master.
    import rospy
    import rostopic
    rospy.init_node('traffic_light_runtime_timing', anonymous=True, disable_signals=True)
    stop = threading.Event()
    signals = []
    def terminate(number, _frame):
        signals.append(number)
        stop.set()
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, terminate)
    topics = dict(TOPICS, front_image=args.front_topic)
    published = dict(rospy.get_published_topics())
    if args.front_topic == TOPICS['front_image'] and args.front_topic not in published:
        alternate = '/sensor/camera/front/image_raw/compressed'
        if alternate in published:
            topics['front_image'] = alternate
    stats = {key: TopicStats(expect_stamp=key != 'control_command') for key in topics}
    lock, subscribers, sampler = threading.Lock(), {}, SystemSampler()
    started, next_discovery = time.monotonic(), 0.
    def callback(message, key):
        receipt_mono, receipt_ros = time.monotonic(), rospy.Time.now().to_sec()
        with lock:
            stats[key].observe(message, receipt_ros, receipt_mono)
    def snapshot():
        now = time.monotonic()
        with lock:
            observed = {key: value.snapshot(now) for key, value in stats.items()}
        return dict(utc=datetime.now(timezone.utc).isoformat(), elapsed_sec=now - started,
                    topics=observed, system=sampler.sample(now, output))
    metadata = dict(topics=topics, passive=True, duration_sec=args.duration_sec,
                    notes='No payloads, images, command arguments or publishers. Header age is not isolated compute latency; clocks must agree. CPU values use host /proc; per-process 100% is one core.')
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    try:
        with (output / 'samples.jsonl').open('x', buffering=1) as stream:
            while not stop.is_set() and not rospy.is_shutdown() and time.monotonic() - started < args.duration_sec:
                if time.monotonic() >= next_discovery:
                    for key, topic in topics.items():
                        if key not in subscribers:
                            message_class, _, _ = rostopic.get_topic_class(topic, blocking=False)
                            if message_class:
                                subscribers[key] = rospy.Subscriber(topic, message_class, callback,
                                                                     callback_args=key, queue_size=100)
                    next_discovery = time.monotonic() + 5.
                stream.write(json.dumps(snapshot(), allow_nan=False) + '\n')
                stop.wait(1.)
    finally:
        for subscriber in subscribers.values():
            subscriber.unregister()
        summary = snapshot()
        summary.update(termination_signals=signals, subscribed_keys=sorted(subscribers))
        (output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
        rospy.signal_shutdown('passive timing observer completed')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
