#!/usr/bin/env python3
"""Korean, bounded-rate terminal view. Raw output is never severity-filtered.

This is an operator display, NOT a topic watchdog or a driving safety decision.
run owns only its child process group; filter owns no processes. No ROS imports.
"""
import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import re
import selectors
import signal
import subprocess
import sys
import time
import unicodedata


LABELS = {
    'roscore': 'ROS', 'udp': '통신', 'udp_lan': 'LAN통신', 'sensing': '센서TF',
    'localization': '측위', 'bevfusion': '객체검출', 'tracking': '추적·예측',
    'traffic': '신호등', 'planning': '판단', 'planviz': '지도화면', 'control': '제어',
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
            return ''.join(result) + '…'
        result.append(char)
        used += size
    return ''.join(result)


@dataclass(frozen=True)
class Event:
    code: str
    level: str
    title: str
    detail: str


# Match actual source signatures first. Severity here is operational display
# importance, not the original ROS severity. Unmatched errors remain visible.
RULES = [
    ('raw_lost', 'ERROR', '원문 저장 실패·실행 유지', r'원본 로그 저장 실패'),
    ('ego_off', 'INFO', 'Ego GT 미사용 설정', r'Skipping Ego Vehicle Status UDP receiver'),
    ('gnss_hold', 'WARN', 'GPS 높이 유지·XY 추정 계속', r'Holding last valid GNSS z'),
    ('gnss_resume', 'INFO', 'GPS 높이 수신 복귀', r'Fresh GNSS z resumed'),
    ('trt_fallback', 'WARN', 'TRT 미사용·동일 모델 일반 추론', r'optional TRT image is unavailable'),
    ('corridor_failed', 'ERROR', '구간 필터 우회·위치/지도 확인', r'vehicle.only corridor FAIL.OPEN'),
    ('vehicle_only', 'INFO', '구간 차량 전용 정책', r'vehicle.only corridor (?:ON|OFF)|removed \d+ nonvehicle predicted objects'),
    ('sync', 'WARN', '카메라·LiDAR 시각 불일치·입력 폐기', r'BEVFusion sync rejected'),
    ('preprocess', 'ERROR', '센서 전처리 실패', r'BEVFusion preprocessing failed'),
    ('inference', 'ERROR', '추론 서버 요청 실패', r'BEVFusion request failed'),
    ('duplicate', 'ERROR', '검출 발행 중복·실행 거부', r'Another detector already publishes'),
    ('cnn', 'ERROR', '신호 CNN 실행 불가', r'CNN unavailable; missing observations'),
    ('weights', 'ERROR', '가중치 없음·설치 확인', r'가중치 파일 없음|checkpoint.*(?:not found|does not exist)'),
    ('disk', 'ERROR', '저장 공간 부족', r'No space left on device|Disk quota exceeded'),
    ('gpu', 'ERROR', 'GPU 메모리/실행 오류', r'CUDA.*(?:out of memory|error)|CUDNN_STATUS|CUBLAS_STATUS|NVIDIA.*failed'),
    ('port', 'ERROR', '포트 점유·중복 실행 확인', r'Address already in use|EADDRINUSE'),
    ('master', 'ERROR', 'ROS master 연결 실패', r'Unable to contact.*master|cannot contact.*master|Failed to contact master'),
    ('map', 'ERROR', '지도 파일/버전 오류', r'map.*(?:does not exist|not found|mismatch|v99.only)|SHA.*mismatch'),
    ('network', 'ERROR', '네트워크 송수신 오류', r'Ctrl Cmd UDP send failed|UDP poll failed|Connection refused|Network is unreachable'),
    ('stale', 'WARN', '입력 오래됨·수신/부하 확인', r'local odom.*stale|stale local state|stale or unavailable trajectory/state|Waiting for fresh local odometry'),
    ('mpt_overlap', 'ERROR', '장애물 여유영역과 차체 겹침·정지', r'\[MPT\] Measured footprint already overlaps static obstacle clearance'),
    ('mpt_emergency', 'ERROR', '궤적 최적화 실패·긴급제동 출력', r'\[MPT\] Optimization failed; emergency brake'),
    ('mpt_braking', 'WARN', '궤적 최적화 실패·기존 경로로 제동', r'\[MPT\] Optimization failed; braking on previous path'),
    ('no_path', 'WARN', '판단 경로 없음·경로/측위 확인', r'BPP suppressed output|BPP has no reference output|behavior path output is empty'),
    ('signal_bypass', 'WARN', '신호 미수신·기존 GO 우회 중', r'missing/stale signal: competition fail.open GO'),
    ('velocity_wait', 'INFO', '속도 판단 입력 대기', r'motion_velocity_planner waits for required'),
    ('tf', 'WARN', '좌표변환 시각/연결 확인', r'LookupException|ExtrapolationException|Could not.*transform|TF_REPEATED_DATA|TF_OLD_DATA'),
    ('roi', 'WARN', '미션 신호 선택 미확인', r'\[Traffic map ROI\] missing|route_missing_or_stale|no_visible_route_head'),
    ('traceback', 'ERROR', '프로그램 예외·원문 확인', r'Traceback \(most recent call last\)|(?:^|[\s.])(?-i:(?:[A-Z][A-Za-z0-9]*)?(?:Error|Exception)):'),
    ('died', 'ERROR', '노드 종료·원문 확인', r'process has died|process.*died|Segmentation fault|terminate called|RLException'),
]
COMPILED = [(code, level, title, re.compile(pattern, re.I)) for code, level, title, pattern in RULES]


def classify(line, component=''):
    text = clean(line)
    if re.match(r'^\* /[^ ]+:', text):
        return Event('setup', 'INFO', '기동 설정 목록', text)
    for code, level, title, pattern in COMPILED:
        if pattern.search(text):
            # A real ROS error must not be demoted by a benign text match.
            if level != 'ERROR' and re.search(r'\[(?:ERROR|FATAL)\]|^(?:ERROR|FATAL)\b', text):
                return Event(code, 'ERROR', title, text)
            return Event(code, level, title, text)
    if re.search(r'\[(?:ERROR|FATAL)\]|^(?:ERROR|FATAL)\b', text, re.I):
        return Event('error', 'ERROR', '오류·원문 확인', text)
    if re.search(r'\[(?:WARN|WARNING)\]|^(?:WARN|WARNING)\b', text, re.I):
        return Event('warning', 'WARN', '경고·원문 확인', text)
    return Event('info', 'INFO', '최근 출력', text)


def event_text(event):
    """Known signatures get Korean explanations; unknown diagnostics stay visible."""
    text = re.sub(r'^\[(?:INFO|WARN|WARNING|ERROR|FATAL|DEBUG)\]\s*(?:\[[^\]]+\]\s*)?', '', event.detail)
    if event.code == 'ego_off':
        return '대회 설정·/Ego_topic 미사용'
    if event.code == 'gnss_hold':
        age = re.search(r'age\s*[:=]?\s*([\d.]+)', text)
        return event.title + (' | 높이 age=' + age[1] + '초' if age else '')
    if event.code == 'gnss_resume':
        return event.title
    if event.code == 'sync':
        difference = re.search(r'differs by\s*([\d.]+)', text)
        return event.title + (' | 차이=' + difference[1] + '초' if difference else '')
    if event.code == 'trt_fallback':
        return event.title + '·추론 속도 확인'
    if event.code == 'vehicle_only':
        state = re.search(r'corridor\s+(ON|OFF)', text, re.I)
        return event.title + (' | ' + state[1].upper() if state else '·비차량 출력 제외')
    if event.code == 'signal_bypass':
        return event.title + '·녹색 관측을 뜻하지 않음'
    if event.code == 'roi':
        return event.title + '·원거리/통과후/경로 준비 여부 확인'
    if event.code == 'no_path':
        return event.title
    if event.code in ('mpt_overlap', 'mpt_emergency', 'mpt_braking'):
        if event.code == 'mpt_overlap':
            return event.title + '·실제 충돌 확정 아님·검출/추적 확인'
        clearance = re.search(r'(?:clear distance\s+|clear=)([\d.+-]+)', text)
        return event.title + (' | 기존 경로 여유=' + clearance[1] + 'm' if clearance else '')
    if event.code == 'setup':
        return '기동 설정 출력·오류 아님'
    if event.code == 'info':
        if re.search(r'waiting for route', text, re.I):
            return '경로 요청 대기'
        # Keep the observed lamp tokens, without inventing planner GO/STOP.
        traffic = re.search(r'\[Traffic\]\s*(.*)', event.detail)
        if traffic:
            return '신호 관측 ' + traffic[1]
        return text
    return event.title + ' | ' + text


class Summary:
    def __init__(self, component, width=120):
        self.component, self.width = LABELS.get(component, component), width
        self.counts = Counter()
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
        self.last = event
        if event.code == 'raw_lost':
            self.raw_lost = True
        if event.level == 'ERROR':
            self.total_errors += 1
            self.last_error_time = datetime.now().strftime('%H:%M:%S')
        rank = {'INFO': 0, 'WARN': 1, 'ERROR': 2}
        if self.important is None or rank[event.level] >= rank[self.important.level]:
            self.important = event
        # At most three new error types per interval, repeats counted in render.
        if event.level == 'ERROR' and event.code not in self.immediate and len(self.immediate) < 3:
            self.immediate.add(event.code)
            return self.line('오류', event_text(event))
        return None

    def line(self, state, detail):
        prefix = datetime.now().strftime('%H:%M:%S') + ' [' + self.component + '] ' + state + ' | '
        return clipped(prefix + detail, self.width)

    def render(self, now=None):
        now = time.monotonic() if now is None else now
        event = self.important
        counts = self.counts
        history = ('누적오류%d(%s) | ' % (self.total_errors, self.last_error_time)) if self.total_errors else ''
        if self.raw_lost:
            history = '원문 저장 실패 지속 | ' + history
        if event is None:
            state = '출력 대기'
            detail = history + '새 로그 없음 %.1f초·토픽 정상 여부 미확인' % max(0, now - self.last_activity)
        else:
            state = {'INFO': '출력 수신', 'WARN': '주의', 'ERROR': '오류 발생'}[event.level]
            detail = history + '이번 구간 오류%d/주의%d | ' % (counts['ERROR'], counts['WARN'])
            detail += event_text(event)
        result = self.line(state, detail)
        self.counts.clear()
        self.important = None
        self.immediate.clear()
        return result


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
                    color = '91' if '] 오류' in line else ('93' if '] 주의' in line else '37')
                    rendered = '\x1b[' + color + 'm' + line + '\x1b[0m'
                stream.write(rendered + '\n')
                stream.flush()
                return
            except (OSError, ValueError):
                self.disabled.add(name)


class RawSink:
    """Keep the stack running on mid-run disk failure; visibly retain that fact."""
    def __init__(self, handle, summary, emit):
        self.handle, self.summary, self.emit = handle, summary, emit
        self.failed = False
        self.closed = False

    def _failure(self, error):
        if not self.failed:
            self.failed = True
            message = '[ERROR] 원본 로그 저장 실패·실행은 유지·이후 원문 보존 불가: ' + str(error)
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
                    immediate = summary.ingest('[ERROR] 실행 명령 종료코드 %d·남은 출력 마감 중' % code)
                    if immediate:
                        emit(immediate)
            # stdout can remain open after roslaunch exits. Do not hide that
            # exit indefinitely behind a perfectly quiet child/grandchild pipe.
            signalled = signal_time is not None and signal_time[0] is not None
            if parent_exit_time is not None and not signalled:
                elapsed = now - parent_exit_time
                if elapsed > stop_timeout and not terminate_sent:
                    emit(summary.line('종료 정리', '상위 명령 종료·이 실행의 남은 자식만 종료 요청'))
                    stop_group(child, signal.SIGTERM)
                    terminate_sent = True
                if elapsed > 2 * stop_timeout and kill_time is None:
                    stop_group(child, signal.SIGKILL)
                    kill_time = now
            if signalled and now - signal_time[0] > stop_timeout and kill_time is None:
                stop_group(child, signal.SIGKILL)
                kill_time = now
            if kill_time is not None and now - kill_time > .5:
                immediate = summary.ingest('[ERROR] 종료 후 출력 파이프 미마감·분리된 프로세스는 별도 확인')
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
        raise ValueError('실행할 명령이 없음')
    path, handle = new_log(args.log_dir, args.component)
    emit = Screen()
    emit('원본 로그: ' + str(path))
    summary = Summary(args.component, args.width)
    raw = RawSink(handle, summary, emit)
    child = None
    old_handlers = {}
    signal_time = [None]

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
            emit(summary.line('입력 종료', '필터 EOF·실행 명령의 종료코드는 호출 shell에서 확인'))
            return 2 if raw.failed else 0
        command = args.command[1:] if args.command[0] == '--' else args.command
        if not command:
            raise ValueError('실행할 명령이 없음')
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 env=env, start_new_session=True)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old_handlers[sig] = signal.signal(sig, forward)
        emit(summary.line('시작', '프로세스 실행·토픽 정상 여부는 별도 확인'))
        drain(child.stdout.fileno(), raw, summary, args.interval, emit,
              child, args.stop_timeout, signal_time)
        code = child.wait()
        result = code if code >= 0 else 128 - code
        raw.close()
        if raw.failed and result == 0:
            result = 2  # Evidence failure, not an algorithm error or a stop command.
        end = ('종료코드 %d' % result) + ('·오류 원문 확인' if result else '·주행 안전 판정 아님')
        emit(summary.line('종료', end))
        return result
    finally:
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


def demo(args):
    path, raw = new_log(args.log_dir, 'demo')
    print('모의 출력 예시·실주행/실측 아님 | 원본 로그: ' + str(path))
    examples = [
        ('udp', '[WARN] Skipping Ego Vehicle Status UDP receiver (type 770) by configuration'),
        ('localization', '[WARN] Holding last valid GNSS z: age 2.5; planar fused state remains valid'),
        ('bevfusion', '[WARN] BEVFusion sync rejected: front differs by 0.137 sec'),
        ('traffic', '[INFO] [Traffic] B | gc | GO'),
        ('planning', '[INFO] waiting for route'),
        ('planning', '[WARN] [MPT] Optimization failed; braking on previous path (clear=59.20 stop=58.20)'),
        ('planning', '[WARN] [MPT] Measured footprint already overlaps static obstacle clearance; stopping'),
        ('control', '[WARN] stale or unavailable trajectory/state'),
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
        command.add_argument('--interval', type=float, default=1.)
        command.add_argument('--width', type=int, default=120)
        command.add_argument('--stop-timeout', type=float, default=20.)
        if mode == 'run':
            command.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not math.isfinite(args.interval) or args.interval < .05:
        parser.error('--interval must be finite and >= 0.05 seconds')
    if not math.isfinite(args.stop_timeout) or args.stop_timeout <= 0:
        parser.error('--stop-timeout must be finite and positive')
    if args.width < 60:
        parser.error('--width must be >= 60 columns')
    try:
        return demo(args) if args.mode == 'demo' else run(args)
    except (OSError, ValueError) as error:
        print('현장 로그 오류: ' + clean(str(error)), file=sys.stderr, flush=True)
        return 127 if isinstance(error, FileNotFoundError) else 1
    except KeyboardInterrupt:
        return 130


def component_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', value):
        raise argparse.ArgumentTypeError('invalid component name')
    return value


if __name__ == '__main__':
    sys.exit(main())
