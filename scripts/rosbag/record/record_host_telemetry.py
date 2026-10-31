#!/usr/bin/env python3
"""Read-only, ROS-independent Linux host telemetry for a paired LAN recording.

This measures host counters, not sensor payload loss or clock synchronization.
Run one copy on each PC and keep both JSONL files with the bag. No network,
service, kernel, GPU, or process settings are modified. Python 3.8+ stdlib only.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import signal
import socket
import subprocess
import threading
import time

SCHEMA = 'ajounice.host_telemetry.v1'
CPU_FIELDS = ('user', 'nice', 'system', 'idle', 'iowait', 'irq', 'softirq', 'steal',
              'guest', 'guest_nice')
NIC_FIELDS = ('rx_bytes', 'rx_packets', 'rx_errors', 'rx_dropped',
              'tx_bytes', 'tx_packets', 'tx_errors', 'tx_dropped')
UDP_FIELDS = ('InDatagrams', 'OutDatagrams', 'InErrors', 'RcvbufErrors',
              'SndbufErrors', 'InCsumErrors', 'NoPorts', 'IgnoredMulti')
GPU_FIELDS = ('index', 'uuid', 'utilization.gpu', 'utilization.memory',
              'memory.used', 'memory.total', 'temperature.gpu', 'power.draw')


def utc_now(wall_ns):
    return datetime.datetime.fromtimestamp(wall_ns / 1e9, datetime.timezone.utc).isoformat()


def error_status(error):
    # Do not include a subprocess command, environment, or arbitrary file text.
    return {'status': 'unavailable', 'error_type': type(error).__name__,
            'errno': getattr(error, 'errno', None)}


def counter_delta(current, previous, elapsed_sec):
    """Keep cumulative values and mark baseline/reset, never invent zero rates."""
    reset = []
    delta = {}
    rate = {}
    for key, value in current.items():
        old = previous.get(key) if previous is not None else None
        if old is not None and value < old:
            reset.append(key)
        valid = old is not None and value >= old and elapsed_sec is not None and elapsed_sec > 0
        delta[key] = value - old if valid else None
        rate[key] = delta[key] / elapsed_sec if valid else None
    return {'counters': current, 'delta': delta, 'per_second': rate,
            'baseline': previous is None, 'counter_reset': bool(reset), 'reset_fields': reset}


def parse_cpu(text):
    result = {}
    for line in text.splitlines():
        parts = line.split()
        if parts and (parts[0] == 'cpu' or parts[0][3:].isdigit() and parts[0].startswith('cpu')):
            values = list(map(int, parts[1:]))
            if len(values) < 4 or any(value < 0 for value in values):
                raise ValueError('Invalid CPU counters')
            result[parts[0]] = dict(zip(CPU_FIELDS, values))
    if 'cpu' not in result:
        raise ValueError('No aggregate CPU counters')
    return result


def parse_meminfo(text):
    result = {}
    for line in text.splitlines():
        key, _, value = line.partition(':')
        parts = value.split()
        if parts and key in ('MemTotal', 'MemFree', 'MemAvailable', 'Buffers', 'Cached',
                             'SwapTotal', 'SwapFree', 'Dirty', 'Writeback'):
            if len(parts) != 2 or parts[1] != 'kB':
                raise ValueError('Unexpected meminfo unit')
            result[key + '_bytes'] = int(parts[0]) * 1024
    if 'MemTotal_bytes' not in result:
        raise ValueError('No memory counters')
    return result


def parse_interfaces(text):
    result = {}
    for line in text.splitlines():
        name, sep, values = line.partition(':')
        if not sep:
            continue
        parts = values.split()
        if len(parts) < 16:
            raise ValueError('Invalid interface counters')
        result[name.strip()] = dict(zip(NIC_FIELDS, (int(parts[i]) for i in (0, 1, 2, 3, 8, 9, 10, 11))))
    return result


def parse_udp(text):
    lines = text.splitlines()
    for index, line in enumerate(lines[:-1]):
        header = line.split()
        values = lines[index + 1].split()
        if header and header[0] == 'Udp:' and values and values[0] == 'Udp:':
            if len(header) != len(values):
                raise ValueError('UDP header/value length mismatch')
            available = dict(zip(header[1:], map(int, values[1:])))
            # Unsupported fields stay absent, rather than appearing to be zero.
            return {key: available[key] for key in UDP_FIELDS if key in available}
    raise ValueError('No Linux UDP counters')


def parse_udp_sockets(text, ports, family):
    """Only port, inode, queue sizes, and drops; no addresses, UID, or payload."""
    result = {}
    for line in text.splitlines()[1:]:
        parts = line.split()
        if not parts:
            continue
        if len(parts) < 13:
            raise ValueError('Socket table does not expose sk_drops')
        port = int(parts[1].rsplit(':', 1)[1], 16)
        if port not in ports:
            continue
        tx_queue, rx_queue = (int(value, 16) for value in parts[4].split(':'))
        inode = int(parts[9])
        result[family + ':' + str(inode)] = {
            'family': family, 'local_port': port, 'inode': inode,
            'rx_queue_bytes': rx_queue, 'tx_queue_bytes': tx_queue,
            'sk_drops': int(parts[-1]),
        }
    return result


def parse_process_stat(text):
    # The comm field may itself contain spaces or ')' characters.
    tail = text[text.rindex(')') + 1:].split()
    if len(tail) < 22:
        raise ValueError('Incomplete process stat')
    return {'cpu_ticks': int(tail[11]) + int(tail[12]), 'start_ticks': int(tail[19]),
            'virtual_memory_bytes': int(tail[20]), 'rss_pages': int(tail[21]), 'state': tail[0]}


class HostSampler:
    """Independent snapshots; deltas use monotonic time, not wall-clock time."""
    def __init__(self, proc_root=Path('/proc'), sys_root=Path('/sys'), interfaces=None,
                 pids=None, udp_ports=None, gpu_interval=5.0, gpu_timeout=1.0,
                 role='algorithm', gpu=True, session_id=None):
        self.proc_root = Path(proc_root)
        self.sys_root = Path(sys_root)  # Reserved for compatible caller configuration.
        self.interfaces = None if interfaces is None else sorted(set(interfaces))
        self.pids = sorted(set(pids or []))
        self.udp_ports = sorted(set(udp_ports or []))
        self.gpu_interval = gpu_interval
        self.gpu_timeout = gpu_timeout
        self.role = role
        self.session_id = session_id
        self.hostname = socket.gethostname()
        machine_id = ''
        try:
            machine_id = Path('/etc/machine-id').read_text().strip()
        except OSError:
            pass
        self.host_id = hashlib.sha256((self.hostname + '\0' + machine_id).encode()).hexdigest()[:16]
        self.boot_id_hash = None
        try:
            self.boot_id_hash = hashlib.sha256(self._read('sys/kernel/random/boot_id').strip().encode()).hexdigest()[:16]
        except OSError:
            pass
        self.clock_id = self.host_id + ':' + (self.boot_id_hash or 'unknown-boot') + ':CLOCK_MONOTONIC'
        self.gpu_enabled = gpu
        self.clock_ticks = os.sysconf('SC_CLK_TCK')
        self.page_size = os.sysconf('SC_PAGE_SIZE')
        self._previous = {}
        self._last_monotonic_ns = None
        self._last_gpu_ns = None
        self._gpu_result = None
        self._gpu_binary = shutil.which('nvidia-smi') if gpu else None
        self._sequence = 0

    def _read(self, relative):
        return (self.proc_root / relative).read_text()

    def provenance(self):
        return {'record_type': 'metadata', 'schema': SCHEMA, 'role': self.role,
                'session_id': self.session_id,
                'host_id': self.host_id, 'hostname': self.hostname,
                'boot_id_hash': self.boot_id_hash, 'clock_id': self.clock_id,
                'os': {'system': platform.system(), 'release': platform.release(),
                       'machine': platform.machine(), 'python': platform.python_version()},
                'clocks': {'wall': 'time.time_ns/CLOCK_REALTIME',
                           'monotonic': time.get_clock_info('monotonic').implementation,
                           'monotonic_resolution_sec': time.get_clock_info('monotonic').resolution,
                           'cross_host_synchronization_verified': False},
                'config': {'interfaces': self.interfaces, 'pids': self.pids,
                           'udp_ports': self.udp_ports, 'gpu_enabled': self.gpu_enabled,
                           'gpu_interval_sec': self.gpu_interval, 'gpu_timeout_sec': self.gpu_timeout,
                           'cpu_clock_ticks_per_second': self.clock_ticks, 'page_size_bytes': self.page_size},
                'limitations': ['Host/global counters do not identify a specific sensor or packet sequence.',
                                'NIC counters cover an interface; UDP counters cover all UDP in this network namespace.',
                                'Socket sk_drops cover only selected port/inode sockets, not source sensor sequence loss.',
                                'No UDP payload, full process command, environment, or credentials are recorded.',
                                'Per-process CPU percent uses one CPU as 100%; multithreaded tasks may exceed 100%.',
                                'A timestamp alone does not establish cross-PC clock alignment.']}

    def _counter(self, key, values, elapsed):
        result = counter_delta(values, self._previous.get(key), elapsed)
        self._previous[key] = values
        return result

    def _forget(self, prefix):
        for key in list(self._previous):
            if key.startswith(prefix):
                del self._previous[key]

    def _cpu(self, elapsed):
        try:
            rows = parse_cpu(self._read('stat'))
            result = {}
            for name, values in rows.items():
                counter = self._counter('cpu:' + name, values, elapsed)
                deltas = counter['delta']
                # Linux guest times are already included in user/nice.
                keys = [key for key in CPU_FIELDS[:8] if key in values]
                valid = all(deltas[key] is not None for key in keys)
                total = sum(deltas[key] for key in keys) if valid else None
                idle = deltas.get('idle', 0) + deltas.get('iowait', 0) if valid else None
                counter['utilization_percent'] = 100 * (total - idle) / total if total else None
                result[name] = counter
            # A removed CPU must not reuse an old baseline if it reappears.
            for key in list(self._previous):
                if key.startswith('cpu:') and key[4:] not in rows:
                    del self._previous[key]
            return {'status': 'ok', 'units': 'kernel_clock_ticks', 'cpus': result}
        except (OSError, ValueError) as error:
            self._forget('cpu:')
            return error_status(error)

    def _memory(self):
        try:
            return dict(status='ok', **parse_meminfo(self._read('meminfo')))
        except (OSError, ValueError) as error:
            return error_status(error)

    def _load(self):
        try:
            parts = self._read('loadavg').split()
            running, total = map(int, parts[3].split('/'))
            return {'status': 'ok', 'load_1m': float(parts[0]), 'load_5m': float(parts[1]),
                    'load_15m': float(parts[2]), 'runnable_tasks': running, 'tasks': total}
        except (OSError, ValueError, IndexError) as error:
            return error_status(error)

    def _network(self, elapsed):
        try:
            rows = parse_interfaces(self._read('net/dev'))
            selected = rows if self.interfaces is None else self.interfaces
            result = {}
            for name in selected:
                if name not in rows:
                    result[name] = {'status': 'unavailable', 'error_type': 'InterfaceNotPresent'}
                    self._previous.pop('nic:' + name, None)
                else:
                    result[name] = dict(status='ok', **self._counter('nic:' + name, rows[name], elapsed))
            for key in list(self._previous):
                if key.startswith('nic:') and key[4:] not in rows:
                    del self._previous[key]
            return {'status': 'ok', 'interfaces': result}
        except (OSError, ValueError) as error:
            self._forget('nic:')
            return error_status(error)

    def _udp(self, elapsed):
        try:
            values = parse_udp(self._read('net/snmp'))
            return dict(status='ok', **self._counter('udp', values, elapsed))
        except (OSError, ValueError) as error:
            self._previous.pop('udp', None)
            return error_status(error)

    def _udp_sockets(self, elapsed):
        if not self.udp_ports:
            return {'status': 'disabled', 'reason': 'No local UDP ports selected'}
        result = {}
        sources = {}
        for table, family in (('net/udp', 'ipv4'), ('net/udp6', 'ipv6')):
            try:
                rows = parse_udp_sockets(self._read(table), self.udp_ports, family)
                sources[family] = {'status': 'ok'}
                for key, row in rows.items():
                    delta = self._counter('socket:' + key, {'sk_drops': row['sk_drops']}, elapsed)
                    result[key] = dict(row, sk_drops_delta=delta['delta']['sk_drops'],
                                       sk_drops_per_second=delta['per_second']['sk_drops'],
                                       counter_reset=delta['counter_reset'])
            except (OSError, ValueError) as error:
                sources[family] = error_status(error)
        # Socket disappearance/recreation is not reported as zero packet loss.
        for key in list(self._previous):
            if key.startswith('socket:') and key[7:] not in result:
                del self._previous[key]
        return {'status': 'ok' if all(row['status'] == 'ok' for row in sources.values()) else 'partial',
                'sources': sources, 'selected_local_ports': self.udp_ports,
                'ports_without_visible_socket': sorted(set(self.udp_ports) - {row['local_port'] for row in result.values()}),
                'sockets': result}

    def _processes(self, elapsed):
        result = {}
        for pid in self.pids:
            key = 'pid:' + str(pid)
            try:
                values = parse_process_stat(self._read(str(pid) + '/stat'))
                previous = self._previous.get(key)
                same_process = previous is not None and previous['start_ticks'] == values['start_ticks']
                cpu_delta = counter_delta({'cpu_ticks': values['cpu_ticks']},
                                          {'cpu_ticks': previous['cpu_ticks']} if same_process else None, elapsed)
                rate = cpu_delta['per_second']['cpu_ticks']
                result[str(pid)] = {
                    'status': 'ok', 'pid': pid, 'start_ticks': values['start_ticks'], 'state': values['state'],
                    'cpu_ticks': values['cpu_ticks'], 'cpu_percent': rate / self.clock_ticks * 100 if rate is not None else None,
                    'rss_bytes': values['rss_pages'] * self.page_size,
                    'virtual_memory_bytes': values['virtual_memory_bytes'],
                    'baseline': not same_process, 'pid_reused': previous is not None and not same_process,
                    'counter_reset': cpu_delta['counter_reset'],
                }
                self._previous[key] = values
            except (OSError, ValueError) as error:
                result[str(pid)] = dict(pid=pid, **error_status(error))
                self._previous.pop(key, None)
        return result

    def _gpu(self, monotonic_ns):
        if not self.gpu_enabled:
            return {'status': 'disabled'}
        if not self._gpu_binary:
            return {'status': 'unavailable', 'error_type': 'NvidiaSmiNotFound'}
        if self._last_gpu_ns is None or (monotonic_ns - self._last_gpu_ns) / 1e9 >= self.gpu_interval:
            self._last_gpu_ns = monotonic_ns
            try:
                completed = subprocess.run(
                    [self._gpu_binary, '--query-gpu=' + ','.join(GPU_FIELDS), '--format=csv,noheader,nounits'],
                    capture_output=True, text=True, timeout=self.gpu_timeout, check=False,
                )
                if completed.returncode:
                    self._gpu_result = {'status': 'unavailable', 'error_type': 'NvidiaSmiFailed',
                                        'exit_code': completed.returncode}
                else:
                    rows = []
                    for line in completed.stdout.splitlines():
                        values = [part.strip() for part in line.split(',')]
                        if len(values) != len(GPU_FIELDS):
                            raise ValueError('Unexpected nvidia-smi columns')
                        row = dict(zip(GPU_FIELDS, values))
                        for name in GPU_FIELDS:
                            if name == 'uuid':
                                continue
                            try:
                                value = float(row[name])
                                row[name] = value if math.isfinite(value) else None
                            except ValueError:
                                row[name] = None  # N/A/unsupported is not a measured zero.
                        rows.append(row)
                    self._gpu_result = {'status': 'ok', 'gpus': rows,
                                        'units': {'memory.used': 'MiB', 'memory.total': 'MiB',
                                                  'utilization.gpu': 'percent', 'utilization.memory': 'percent',
                                                  'temperature.gpu': 'C', 'power.draw': 'W'}}
            except (OSError, ValueError, subprocess.TimeoutExpired) as error:
                self._gpu_result = error_status(error)
        return dict(self._gpu_result, age_sec=(monotonic_ns - self._last_gpu_ns) / 1e9,
                    sampled_monotonic_ns=self._last_gpu_ns)

    def sample(self):
        start = time.monotonic_ns()
        wall = time.time_ns()
        elapsed = None if self._last_monotonic_ns is None else (start - self._last_monotonic_ns) / 1e9
        result = {'record_type': 'sample', 'schema': SCHEMA, 'role': self.role,
                  'session_id': self.session_id, 'host_id': self.host_id, 'clock_id': self.clock_id,
                  'sequence': self._sequence, 'wall_time_ns': wall, 'monotonic_ns': start,
                  'utc': utc_now(wall), 'elapsed_sec': elapsed,
                  'cpu': self._cpu(elapsed), 'load': self._load(), 'memory': self._memory(),
                  'network': self._network(elapsed), 'udp': self._udp(elapsed),
                  'udp_sockets': self._udp_sockets(elapsed), 'processes': self._processes(elapsed),
                  'gpu': self._gpu(start)}
        result['sampling_duration_sec'] = (time.monotonic_ns() - start) / 1e9
        self._last_monotonic_ns = start
        self._sequence += 1
        return result


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path, help='New JSONL file; existing files are never overwritten')
    parser.add_argument('--role', default='algorithm', help='Host role, e.g. algorithm or morai')
    parser.add_argument('--session-id', help='Optional shared recording ID to associate both PCs with the bag')
    parser.add_argument('--duration', type=float, default=0, help='Seconds; 0 waits for Ctrl-C/SIGTERM')
    parser.add_argument('--interval', type=float, default=1, help='Host sample interval seconds, minimum 0.2')
    parser.add_argument('--pid', type=int, action='append', default=[], help='Optional process ID, repeatable')
    parser.add_argument('--interfaces', help='Optional comma-separated NIC names; default all interfaces')
    parser.add_argument('--udp-port', type=int, action='append', default=[], help='Local UDP socket port to inspect; repeatable')
    parser.add_argument('--gpu-interval', type=float, default=5, help='nvidia-smi query interval seconds, minimum 1')
    parser.add_argument('--gpu-timeout', type=float, default=1, help='nvidia-smi timeout seconds, maximum 1')
    parser.add_argument('--no-gpu', action='store_true', help='Do not run nvidia-smi')
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if (not math.isfinite(args.duration) or args.duration < 0 or not math.isfinite(args.interval)
            or args.interval < .2 or not math.isfinite(args.gpu_interval) or args.gpu_interval < 1
            or not math.isfinite(args.gpu_timeout) or not 0 < args.gpu_timeout <= 1
            or any(pid <= 0 for pid in args.pid) or any(not 1 <= port <= 65535 for port in args.udp_port)):
        parser.error('Invalid duration, interval, GPU timeout/interval, PID, or UDP port')
    if platform.system() != 'Linux':
        parser.error('This collector currently supports Linux only')
    interfaces = None if args.interfaces is None else [part.strip() for part in args.interfaces.split(',') if part.strip()]
    if interfaces == []:
        parser.error('interfaces must not be empty')
    sampler = HostSampler(interfaces=interfaces, pids=args.pid, udp_ports=args.udp_port,
                          gpu_interval=args.gpu_interval, gpu_timeout=args.gpu_timeout,
                          role=args.role, gpu=not args.no_gpu, session_id=args.session_id)
    stop = threading.Event()
    previous_handlers = {}
    received_signal = None

    def request_stop(signum, frame):
        nonlocal received_signal
        received_signal = signum
        stop.set()

    # Deliberately no mkdir: the caller chooses/creates the destination directory.
    try:
        output = args.output.expanduser().open('x', encoding='utf-8')
    except OSError as error:
        parser.error('Cannot create new output file: ' + type(error).__name__)
    start = time.monotonic()
    samples = 0
    skipped = 0
    with output:
        def emit(record):
            output.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
            output.flush()

        metadata = sampler.provenance()
        metadata['config'].update(interval_sec=args.interval, duration_sec=args.duration)
        emit(metadata)
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_stop)
        due = start
        try:
            while not stop.is_set():
                now = time.monotonic()
                if args.duration and now - start >= args.duration:
                    break
                wait = max(0, due - now)
                if args.duration:
                    wait = min(wait, max(0, start + args.duration - now))
                if stop.wait(wait):
                    break
                now = time.monotonic()
                if args.duration and now - start >= args.duration:
                    break
                sample = sampler.sample()
                sample['schedule_lateness_sec'] = max(0, now - due)
                next_due = due + args.interval
                missed = max(0, int((time.monotonic() - next_due) // args.interval) + 1)
                skipped += missed
                sample['missed_schedule_slots'] = missed
                emit(sample)
                samples += 1
                due = next_due + missed * args.interval
        finally:
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
            emit({'record_type': 'end', 'schema': SCHEMA, 'role': args.role,
                  'session_id': args.session_id, 'host_id': sampler.host_id, 'clock_id': sampler.clock_id,
                  'wall_time_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(),
                  'samples': samples, 'missed_schedule_slots': skipped,
                  'elapsed_sec': time.monotonic() - start, 'signal': received_signal})
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
