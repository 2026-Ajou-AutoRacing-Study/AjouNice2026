#!/usr/bin/env python3
"""Compact English terminal view. Raw output is never severity-filtered.

This is a read-only operator display, NOT a driving safety decision.
run owns only its child process group; filter owns no processes. Optional ROS
observation runs separately from draining the algorithm's lossless raw output.
"""
import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
import math
import json
import os
from pathlib import Path
import re
import selectors
import shutil
import signal
import subprocess
import sys
import threading
import time
import unicodedata


LABELS = {
    'udp': 'UDP', 'udp_lan': 'UDP', 'sensing': 'TF',
    'localization': 'LOC', 'bevfusion': 'OD', 'tracking': 'TRK', 'traffic': 'TL',
    'planviz': 'RVZ',
}
ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]|\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)')


def clean(text):
    text = ANSI.sub('', text)
    return ''.join(c for c in text if not unicodedata.category(c).startswith('C')).strip()


def clipped(text, width):
    result, used = [], 0
    for char in clean(text):
        size = 0 if unicodedata.combining(char) else (2 if unicodedata.east_asian_width(char) in 'WF' else 1)
        if used + size > width - 1:
            return ''.join(result) + '~'
        result.append(char)
        used += size
    return ''.join(result)


def display_width(value=None):
    """A narrow pane or stale width override must never reject a launch."""
    automatic = max(12, min(48, shutil.get_terminal_size((48, 24)).columns))
    try:
        width = int(value) if value is not None else automatic
    except (ValueError, TypeError, OverflowError):
        return automatic
    return max(12, min(width, 4096)) if width > 0 else automatic


@dataclass(frozen=True)
class Event:
    code: str
    level: str
    title: str
    detail: str


# Display only: these rules never change a ROS logger or a vehicle decision.
RULES = [
    ('raw_lost', 'ERROR', 'raw write failed', r'raw log write failed'),
    ('ego_off', 'INFO', 'Ego off', r'Skipping Ego Vehicle Status UDP receiver'),
    ('gnss_hold', 'WARN', 'gps-z hold', r'Holding last valid GNSS z'),
    ('gnss_resume', 'INFO', 'gps-z resumed', r'Fresh GNSS z resumed'),
    ('gnss_advance', 'INFO', 'GNSS advance', r'GNSS advance [\d.]+ms'),
    ('gnss_raw', 'WARN', 'GNSS raw', r'GNSS motion compensation fallback'),
    ('pose_reset', 'WARN', 'pose reset', r'Reset smooth control alignment: translation_discontinuity'),
    ('trt_fallback', 'WARN', 'TRT off', r'optional TRT image is unavailable'),
    ('corridor_failed', 'ERROR', 'map gate failed', r'vehicle.only corridor FAIL.OPEN'),
    ('vehicle_only', 'INFO', 'vehicle-only', r'vehicle.only corridor (?:ON|OFF)|removed \d+ nonvehicle predicted objects'),
    ('sync', 'WARN', 'sync', r'BEVFusion sync rejected'),
    ('preprocess', 'ERROR', 'preprocess failed', r'BEVFusion preprocessing failed'),
    ('inference', 'ERROR', 'inference failed', r'BEVFusion request failed'),
    ('duplicate', 'ERROR', 'detector duplicate', r'Another detector already publishes'),
    ('cnn', 'ERROR', 'CNN unavailable', r'CNN unavailable; missing observations'),
    ('weights', 'ERROR', 'weights missing', r'가중치 파일 없음|checkpoint.*(?:not found|does not exist)'),
    ('disk', 'ERROR', 'disk full', r'No space left on device|Disk quota exceeded'),
    ('gpu', 'ERROR', 'GPU failed', r'CUDA.*(?:out of memory|error)|CUDNN_STATUS|CUBLAS_STATUS|NVIDIA.*failed'),
    ('port', 'ERROR', 'port busy', r'Address already in use|EADDRINUSE'),
    ('master', 'ERROR', 'master unreachable', r'Unable to contact.*master|cannot contact.*master|Failed to contact master'),
    ('map', 'ERROR', 'map invalid', r'map.*(?:does not exist|not found|mismatch|v\d+.only)|SHA.*mismatch'),
    ('network', 'ERROR', 'network failed', r'Ctrl Cmd UDP send failed|UDP poll failed|Connection refused|Network is unreachable'),
    ('start_refused', 'ERROR', 'start refused', r'Refusing official.start|Refusing.*initial.*pose'),
    ('lidar_timeout', 'WARN', 'LiDAR timeout', r'(?:(?:velodyne|lidar).*(?:poll.*timeout|no packets)|poll\(\) timeout)'),
    ('stale', 'WARN', 'input stale', r'local odom.*stale|stale local state|Waiting for fresh local odometry'),
    ('tf', 'WARN', 'TF unavailable', r'LookupException|ExtrapolationException|Could not.*transform|TF_REPEATED_DATA|TF_OLD_DATA|TF lookup failed'),
    ('display', 'ERROR', 'display unavailable', r'could not connect to display|could not load the Qt platform plugin|no Qt platform plugin could be initialized'),
    ('render', 'WARN', 'rendering issue', r'OGRE EXCEPTION|libGL error|failed to create.*render'),
    ('roi', 'WARN', 'roi missing', r'\[Traffic map ROI\] missing|route_missing_or_stale|no_visible_route_head'),
    ('traceback', 'ERROR', 'exception', r'Traceback \(most recent call last\)|(?:^|[\s.])(?-i:(?:[A-Z][A-Za-z0-9]*)?(?:Error|Exception)):'),
    ('died', 'ERROR', 'node exited', r'process has died|process.*died|Segmentation fault|terminate called|RLException'),
]
COMPILED = [(code, level, title, re.compile(pattern, re.I)) for code, level, title, pattern in RULES]


def traffic_state(text):
    traffic = re.search(r'\[Traffic\](?:\[(?:INFO|WARN|ERROR)\])?\s*(.*)', text)
    if not traffic:
        return None
    parts = [part.strip() for part in traffic[1].split('|')]
    if len(parts) < 2 or not re.fullmatch(r'[A-Za-z0-9+_ -]+', parts[1]):
        return None
    mission = {'D-2': 'D', 'D-1': 'D1(skip)', 'none': 'no mission'}.get(parts[0], parts[0])
    if mission == 'no mission':
        return mission
    if mission not in ('A', 'B', 'C', 'D', 'D1(skip)'):
        return None
    # Lamp observation only: this is NOT the planner's GO/STOP decision.
    return mission + ' ' + parts[1].upper()


def classify(line, component=''):
    text = clean(line)
    if re.match(r'^\* /[^ ]+:', text):
        return Event('setup', 'INFO', 'setup', text)
    for code, level, title, pattern in COMPILED:
        if pattern.search(text):
            if level != 'ERROR' and re.search(r'\[(?:ERROR|FATAL)\]|^(?:ERROR|FATAL)\b', text):
                return Event(code, 'ERROR', title, text)
            return Event(code, level, title, text)
    state = traffic_state(text)
    if state is not None:
        level = ('ERROR' if re.search(r'\[(?:ERROR|FATAL)\]', text) else
                 'WARN' if re.search(r'\[(?:WARN|WARNING)\]', text) else 'INFO')
        return Event('traffic_state', level, state, text)
    if re.search(r'\[(?:ERROR|FATAL)\]|^(?:ERROR|FATAL)\b', text, re.I):
        return Event('error', 'ERROR', 'failure', text)
    if re.search(r'\[(?:WARN|WARNING)\]|^(?:WARN|WARNING)\b', text, re.I):
        return Event('warning', 'WARN', 'notice', text)
    return Event('info', 'INFO', 'log', text)


def event_text(event):
    text = re.sub(r'^(?:\[[^\]]+\]\s*)+', '', event.detail)
    text = re.sub(r'^(?:INFO|WARN|WARNING|ERROR|FATAL|DEBUG)\b\s*:?\s*', '', text)
    if event.code == 'gnss_advance':
        amount = re.search(r'GNSS advance ([\d.]+)ms', text)
        return 'GNSS +%sms' % amount[1] if amount else event.title
    if event.code == 'gnss_raw':
        # Sensor data still flows; only the optional advance was skipped.
        return 'GNSS raw; advance skipped'
    if event.code == 'sync':
        difference = re.search(r'differs by\s*([\d.]+)', text)
        return 'sync' + (' %dms' % round(float(difference[1]) * 1000) if difference else '')
    if event.code == 'vehicle_only':
        state = re.search(r'corridor\s+(ON|OFF)', text, re.I)
        return event.title + (' ' + state[1].lower() if state else '')
    if event.code == 'roi':
        mission = re.search(r'missing\s+([ABCD])\b', text)
        reason = ('route/pose?' if 'route_missing_or_stale' in text else
                  'not in view' if 'no_visible_route_head' in text else 'roi missing')
        return (mission[1] + ' ' if mission else '') + reason
    if event.code == 'start_refused':
        distance = re.search(r'([\d.]+)\s*m\s+away', text)
        return 'start refused' + (' %sm' % distance[1] if distance else '')
    if event.code == 'traffic_state':
        return event.title
    if event.code == 'info':
        return 'ready' if re.search(r'\bready\b', text, re.I) else 'no new status'
    if event.code in ('error', 'warning', 'traceback'):
        # Keep unrecognised English diagnostics readable in the narrow pane.
        text = re.sub(r'^\[[^\]]+\]\s*', '', text)
        return text if text.isascii() and text else event.title + '; raw'
    return event.title


class DisplayLine(str):
    """Severity travels separately so the screen needs no WARN/ERROR prefix."""
    def __new__(cls, value, level='INFO'):
        line = super().__new__(cls, value)
        line.level = level
        return line


class Summary:
    def __init__(self, component, width=48, health=None):
        self.component, self.width = LABELS.get(component, component.upper()[:4]), width
        self.health = health
        self.counts = Counter()
        self.code_counts = Counter()
        self.total_errors = 0
        self.raw_lost = False
        self.last_error_time = None
        self.last = None
        self.important = None
        self.last_activity = time.monotonic()
        self.immediate = set()

    def ingest(self, line):
        event = classify(line, self.component)
        if not event.detail:
            return None
        self.last_activity = time.monotonic()
        self.counts[event.level] += 1
        self.code_counts[event.code] += 1
        self.last = event
        if event.code == 'raw_lost':
            self.raw_lost = True
        if event.level == 'ERROR':
            self.total_errors += 1
            self.last_error_time = datetime.now().strftime('%H:%M:%S')
        rank = {'INFO': 0, 'WARN': 1, 'ERROR': 2}
        if self.important is None or rank[event.level] >= rank[self.important.level]:
            self.important = event
        if event.level == 'ERROR' and event.code not in self.immediate and len(self.immediate) < 3:
            self.immediate.add(event.code)
            return self.line('', event_text(event), 'ERROR')
        return None

    def line(self, state, detail='', level='INFO'):
        prefix = datetime.now().strftime('%S') + ' ' + self.component.ljust(3) + ' '
        message = ' '.join(part for part in (state, detail) if part)
        return DisplayLine(clipped(prefix + message, self.width), level)

    def render(self, now=None):
        now = time.monotonic() if now is None else now
        event = self.important
        health = self.health.snapshot() if self.health is not None else None
        if self.raw_lost:
            detail, level = 'raw write failed', 'ERROR'
        elif health is not None and (event is None or
                                    (event.level == 'INFO' and
                                     (health[1] != 'QUIET' or event.code in ('info', 'setup'))) or
                                    (event.code == 'roi' and event.level == 'WARN'
                                     and 'route_missing_or_stale' in event.detail
                                     and health[0].startswith('no mission')
                                     and health[1] == 'INFO')):
            detail, level = health
        elif event is None:
            detail, level = 'no new logs %.0fs' % max(0, now - self.last_activity), 'QUIET'
        else:
            detail, level = event_text(event), event.level
            if event.code == 'info' and detail == 'no new status':
                level = 'QUIET'
            repetitions = self.code_counts[event.code]
            if repetitions > 1 and event.level != 'INFO':
                detail += ' x%d' % repetitions
        if self.total_errors and level != 'ERROR':
            detail += ' | past%d' % self.total_errors
            level = 'WARN'
        result = self.line('', detail, level)
        self.counts.clear()
        self.code_counts.clear()
        self.important = None
        self.immediate.clear()
        return result


class OptionalHealth:
    """Never block the stdout drain on ROS init/master loss or monitor failure."""
    def __init__(self, component):
        self._lock = threading.Lock()
        self._health = None
        self._closed = False
        self._pending = True
        self._thread = threading.Thread(target=self._start, args=(component,), daemon=True)
        self._thread.start()

    def _start(self, component):
        health = None
        try:
            from field_topic_health import create_topic_health
            health = create_topic_health(component)
        except Exception:
            pass
        with self._lock:
            self._pending = False
            if not self._closed:
                self._health = health
                return
        if health is not None:
            try:
                health.close()
            except Exception:
                pass

    def snapshot(self):
        with self._lock:
            health, pending = self._health, self._pending
        if health is None:
            return ('monitor starting' if pending else 'monitor off; see raw', 'QUIET')
        try:
            return health.snapshot()
        except Exception:
            return 'monitor failed; see raw', 'WARN'

    def close(self):
        with self._lock:
            self._closed = True
            health, self._health = self._health, None
        if health is not None:
            try:
                health.close()
            except Exception:
                pass


def new_log(directory, component):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%y%m%d_%H%M%S_%f')
    path = directory / (stamp + '_' + component + '_raw.log')
    # Open/validate before launching anything. No truncation of existing logs.
    return path, path.open('xb')


class Screen:
    """A closed display pipe must not terminate the actual algorithm process."""
    def __init__(self):
        self.disabled = set()

    def __call__(self, line):
        for name in ('stdout', 'stderr'):
            if name in self.disabled:
                continue
            stream = getattr(sys, name)
            try:
                rendered = line
                if (getattr(stream, 'isatty', lambda: False)() and
                        'NO_COLOR' not in os.environ and os.environ.get('TERM') != 'dumb'):
                    color = {'ERROR': '91', 'WARN': '93', 'QUIET': '90'}.get(
                        getattr(line, 'level', 'INFO'), '92')
                    rendered = '\x1b[' + color + 'm' + line + '\x1b[0m'
                stream.write(rendered + '\n')
                stream.flush()
                return
            except (OSError, ValueError):
                self.disabled.add(name)


class StatusScreen(Screen):
    """Save displayed summaries with full time, independently of raw evidence."""
    def __init__(self, raw_path):
        super().__init__()
        self.handle = None
        self.failed = False
        if raw_path is None:
            return
        try:
            self.handle = raw_path.with_name(raw_path.name.replace('_raw.log', '_status.jsonl')).open('x')
        except OSError:
            self.failed = True

    def __call__(self, line):
        super().__call__(line)
        if self.handle is not None:
            try:
                self.handle.write(json.dumps({
                    'time': datetime.now().astimezone().isoformat(timespec='milliseconds'),
                    'level': getattr(line, 'level', 'INFO'), 'display': str(line),
                }) + '\n')
                self.handle.flush()
            except (OSError, ValueError):
                self.failed = True
                self.close()
        if self.failed:
            super().__call__(DisplayLine('status log unavailable; raw separate', 'WARN'))
            self.failed = False  # One notice; never stop the algorithm for this sidecar.

    def close(self):
        handle, self.handle = self.handle, None
        if handle is not None:
            try:
                handle.close()
            except OSError:
                pass


class RawSink:
    """Log storage failure never prevents or stops the algorithm."""
    def __init__(self, handle, summary, emit):
        self.handle, self.summary, self.emit = handle, summary, emit
        self.failed = False
        self.closed = False

    def _failure(self, error):
        if not self.failed:
            self.failed = True
            message = '[ERROR] raw log write failed: ' + str(error)
            immediate = self.summary.ingest(message)
            if immediate:
                self.emit(immediate)

    def write(self, data):
        if not self.failed:
            try:
                self.handle.write(data)
            except OSError as error:
                self._failure(error)

    def flush(self):
        if not self.failed:
            try:
                self.handle.flush()
            except OSError as error:
                self._failure(error)

    def close(self):
        if self.closed:
            return
        if self.handle is None:
            self.closed = True
            return
        self.flush()
        try:
            self.handle.close()
        except OSError as error:
            self._failure(error)
        self.closed = True


def drain(fd, raw, summary, interval, emit, child=None, stop_timeout=20., signal_time=None):
    selector = selectors.DefaultSelector()
    try:
        selector.register(fd, selectors.EVENT_READ)
    except PermissionError:  # redirected regular file (epoll cannot watch it)
        selector.close()
        selector = selectors.SelectSelector()
        selector.register(fd, selectors.EVENT_READ)
    pending = b''
    next_tick = time.monotonic() + interval
    parent_exit_time = None
    terminate_sent = False
    kill_time = None
    stream_eof = False
    while True:
        events = selector.select(min(.2, max(0., next_tick - time.monotonic())))
        if events:
            data = os.read(fd, 65536)
            if not data:
                stream_eof = True
                selector.unregister(fd)
            else:
                raw.write(data)
                pending += data
                lines = pending.split(b'\n')
                pending = lines.pop()
                for line in lines:
                    immediate = summary.ingest(line.decode('utf-8', errors='replace'))
                    if immediate:
                        emit(immediate)
                # Bound retained line memory; raw bytes are still complete.
                if len(pending) > 262144:
                    immediate = summary.ingest(pending.decode('utf-8', errors='replace'))
                    pending = b''
                    if immediate:
                        emit(immediate)
        now = time.monotonic()
        if child is not None:
            code = child.poll()
            if code is not None and parent_exit_time is None:
                parent_exit_time = now
                if code:
                    immediate = summary.ingest('[ERROR] process exit=%d' % code)
                    if immediate:
                        emit(immediate)
            # stdout can remain open after roslaunch exits. Do not hide that
            # exit indefinitely behind a perfectly quiet child/grandchild pipe.
            signalled = signal_time is not None and signal_time[0] is not None
            if parent_exit_time is not None and not signalled:
                elapsed = now - parent_exit_time
                if elapsed > stop_timeout and not terminate_sent:
                    emit(summary.line('closing', 'child process group'))
                    stop_group(child, signal.SIGTERM)
                    terminate_sent = True
                if elapsed > 2 * stop_timeout and kill_time is None:
                    stop_group(child, signal.SIGKILL)
                    kill_time = now
            if signalled and now - signal_time[0] > stop_timeout and kill_time is None:
                stop_group(child, signal.SIGKILL)
                kill_time = now
            if kill_time is not None and now - kill_time > .5:
                immediate = summary.ingest('[ERROR] output pipe still open')
                if immediate:
                    emit(immediate)
                stream_eof = True
        if now >= next_tick:
            emit(summary.render(now))
            raw.flush()
            next_tick = now + interval
        if stream_eof and (child is None or child.poll() is not None):
            if pending:
                immediate = summary.ingest(pending.decode('utf-8', errors='replace'))
                if immediate:
                    emit(immediate)
            emit(summary.render())
            raw.flush()
            break
    selector.close()


def stop_group(child, sig):
    if child is not None:
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            pass


def run(args):
    if args.mode == 'run' and not args.command:
        raise ValueError('command missing')
    log_error = None
    try:
        path, handle = new_log(args.log_dir, args.component)
    except OSError as error:
        path = handle = None
        log_error = error
    emit = StatusScreen(path)
    summary = Summary(args.component, args.width)
    raw = RawSink(handle, summary, emit)
    if log_error is not None:
        raw._failure(log_error)
    else:
        emit(summary.line('raw saved'))
    child = None
    old_handlers = {}
    signal_time = [None]
    health = None

    def forward(sig, frame):
        if signal_time[0] is None:
            signal_time[0] = time.monotonic()
        # A terminal hangup must not detach a still-driving child. Request the
        # same orderly ROS shutdown as Ctrl-C if this wrapper receives SIGHUP.
        stop_group(child, signal.SIGINT if sig == signal.SIGHUP else sig)

    try:
        if args.mode == 'filter':
            drain(sys.stdin.fileno(), raw, summary, args.interval, emit)
            raw.close()
            emit(summary.line('EOF'))
            return 2 if raw.failed else 0
        command = args.command[1:] if args.command[0] == '--' else args.command
        if not command:
            raise ValueError('command missing')
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 env=env, start_new_session=True)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old_handlers[sig] = signal.signal(sig, forward)
        if not getattr(args, 'no_topic_status', False) and args.component in LABELS:
            health = OptionalHealth(args.component)
            summary.health = health
        emit(summary.line('start'))
        drain(child.stdout.fileno(), raw, summary, args.interval, emit,
              child, args.stop_timeout, signal_time)
        code = child.wait()
        result = code if code >= 0 else 128 - code
        raw.close()
        if raw.failed and result == 0:
            result = 2  # Evidence failure, not an algorithm error or a stop command.
        emit(summary.line('exit=%d' % result, level='ERROR' if result else 'INFO'))
        return result
    finally:
        if health is not None:
            health.close()
        if child is not None:
            if child.poll() is None:
                stop_group(child, signal.SIGTERM)
                try:
                    child.wait(timeout=args.stop_timeout)
                except subprocess.TimeoutExpired:
                    stop_group(child, signal.SIGKILL)
                    child.wait()
            # Kill only remaining members of this invocation's group, never a
            # broad pkill/ROS node cleanup. ROS launch has already had grace time.
            if signal_time[0] is not None:
                stop_group(child, signal.SIGKILL)
            child.stdout.close()
        if old_handlers:
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
        raw.close()
        emit.close()


def demo(args):
    path, raw = new_log(args.log_dir, 'demo')
    print('SIMULATED LOGS / no ROS processes started')
    examples = [
        ('udp', '[WARN] Skipping Ego Vehicle Status UDP receiver (type 770) by configuration'),
        ('localization', '[WARN] Holding last valid GNSS z: age 2.5; planar fused state remains valid'),
        ('bevfusion', '[WARN] BEVFusion sync rejected: front differs by 0.137 sec'),
        ('traffic', '[INFO] [Traffic] B | gc | GO'),
        ('sensing', '[WARN] TF lookup failed'),
        ('tracking', '[INFO] vehicle-only corridor ON'),
        ('bevfusion', '[WARN] BEVFusion request failed: Connection refused'),
        ('bevfusion', '[INFO] next callback received'),
    ]
    try:
        summaries = {}
        for component, line in examples:
            raw.write((line + '\n').encode())
            summary = summaries.setdefault(component, Summary(component, args.width))
            immediate = summary.ingest(line)
            if immediate:
                print(immediate)
            print(summary.render())
    finally:
        raw.close()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)
    for mode in ('run', 'filter', 'demo'):
        command = sub.add_parser(mode)
        command.add_argument('--component', default='stack', type=component_name)
        command.add_argument('--log-dir', default=str(Path(__file__).resolve().parents[2] / 'artifacts/field_logs'))
        command.add_argument('--interval', default=1.)
        command.add_argument('--width', type=display_width, default=display_width(),
                             help='display columns only; narrow/invalid values fall back safely')
        command.add_argument('--stop-timeout', default=20.)
        command.add_argument('--no-topic-status', action='store_true',
                             help='disable optional read-only topic observation')
        if mode == 'run':
            command.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    for name, default in (('interval', 1.), ('stop_timeout', 20.)):
        try:
            value = float(getattr(args, name))
            valid = math.isfinite(value) and (value >= .05 if name == 'interval' else value > 0)
        except (ValueError, TypeError):
            valid = False
        if not valid:
            print('Console %s invalid; using %ss' % (name, default), file=sys.stderr)
            value = default
        setattr(args, name, value)
    try:
        return demo(args) if args.mode == 'demo' else run(args)
    except (OSError, ValueError) as error:
        print(clipped('console failed: ' + clean(str(error)), args.width), file=sys.stderr, flush=True)
        return 127 if isinstance(error, FileNotFoundError) else 1
    except KeyboardInterrupt:
        return 130


def component_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', value):
        raise argparse.ArgumentTypeError('invalid component name')
    return value


if __name__ == '__main__':
    sys.exit(main())
