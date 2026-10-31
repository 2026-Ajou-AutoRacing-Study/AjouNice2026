import contextlib
import importlib.util
import io
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('host_telemetry', Path(__file__).with_name('record_host_telemetry.py'))
telemetry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(telemetry)


def cpu_fixture(user=100, idle=300, guest=10):
    row = '{} 0 0 {} 0 0 0 0 {} 0'.format(user, idle, guest)
    return 'cpu ' + row + '\ncpu0 ' + row + '\n'


def network_fixture(rx=100, drops=2):
    return ('Inter-| Receive | Transmit\n face |bytes packets errs drop fifo frame compressed multicast'
            '|bytes packets errs drop fifo colls carrier compressed\n'
            ' eth0: {} 10 1 {} 0 0 0 0 50 5 0 1 0 0 0 0\n'.format(rx, drops))


def udp_fixture(received=10, errors=1):
    return ('Udp: InDatagrams NoPorts InErrors OutDatagrams RcvbufErrors SndbufErrors InCsumErrors\n'
            'Udp: {} 3 {} 20 4 5 6\n'.format(received, errors))


def socket_fixture(drops=3, inode=123, port=2368):
    return (' sl local_address rem_address st tx_queue rx_queue tr tm->when retrnsmt uid timeout inode ref pointer drops\n'
            ' 0: 0100007F:{:04X} 00000000:0000 07 00000004:00000008 00:00000000 00000000 '
            '1000 0 {} 2 0000000000000000 {}\n'.format(port, inode, drops))


def process_fixture(cpu=100, start=20):
    fields = ['S'] + ['0'] * 21
    fields[11], fields[12] = str(cpu), '0'
    fields[19], fields[20], fields[21] = str(start), '123456', '10'
    return '42 (command with ) spaces) ' + ' '.join(fields)


class HostTelemetryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root / 'net').mkdir()
        (self.root / '42').mkdir()
        self.files = {
            'stat': cpu_fixture(), 'meminfo': 'MemTotal: 1000 kB\nMemAvailable: 400 kB\nSwapTotal: 20 kB\nSwapFree: 10 kB\n',
            'loadavg': '1.00 2.00 3.00 1/100 99\n', 'net/dev': network_fixture(),
            'net/snmp': udp_fixture(), 'net/udp': socket_fixture(),
            'net/udp6': socket_fixture(port=9999), '42/stat': process_fixture(),
        }
        for name, value in self.files.items():
            (self.root / name).write_text(value)
        self.sampler = telemetry.HostSampler(proc_root=self.root, pids=[42], udp_ports=[2368],
                                             gpu=False, session_id='test-session')

    def tearDown(self):
        self.temp.cleanup()

    def test_cpu_delta_excludes_double_counted_guest(self):
        first = self.sampler._cpu(None)
        self.assertIsNone(first['cpus']['cpu']['utilization_percent'])
        (self.root / 'stat').write_text(cpu_fixture(user=150, idle=350, guest=30))
        second = self.sampler._cpu(1)
        self.assertEqual(second['cpus']['cpu']['utilization_percent'], 50)
        self.assertEqual(second['cpus']['cpu0']['utilization_percent'], 50)

    def test_counter_reset_is_not_zero_rate(self):
        reset = telemetry.counter_delta({'packets': 2, 'errors': 5}, {'packets': 8, 'errors': 4}, 2)
        self.assertTrue(reset['counter_reset'])
        self.assertEqual(reset['reset_fields'], ['packets'])
        self.assertIsNone(reset['per_second']['packets'])
        self.assertEqual(reset['per_second']['errors'], .5)
        self.assertIsNone(telemetry.counter_delta({'p': 0}, None, None)['per_second']['p'])

    def test_network_and_udp_preserve_cumulative_and_delta(self):
        self.sampler._network(None)
        self.sampler._udp(None)
        (self.root / 'net/dev').write_text(network_fixture(rx=200, drops=4))
        (self.root / 'net/snmp').write_text(udp_fixture(received=30, errors=2))
        network = self.sampler._network(2)['interfaces']['eth0']
        udp = self.sampler._udp(2)
        self.assertEqual(network['per_second']['rx_bytes'], 50)
        self.assertEqual(network['delta']['rx_dropped'], 2)
        self.assertEqual(udp['counters']['RcvbufErrors'], 4)
        self.assertEqual(udp['per_second']['InDatagrams'], 10)
        self.assertNotIn('IgnoredMulti', udp['counters'])

    def test_socket_ports_and_inode_boundaries(self):
        first = self.sampler._udp_sockets(None)
        self.assertEqual(len(first['sockets']), 1)
        row = next(iter(first['sockets'].values()))
        self.assertEqual(row['rx_queue_bytes'], 8)
        self.assertIsNone(row['sk_drops_delta'])
        self.assertNotIn('local_address', row)
        (self.root / 'net/udp').write_text(socket_fixture(drops=5))
        row = next(iter(self.sampler._udp_sockets(1)['sockets'].values()))
        self.assertEqual(row['sk_drops_delta'], 2)
        (self.root / 'net/udp').write_text(socket_fixture(drops=0, inode=456))
        row = next(iter(self.sampler._udp_sockets(1)['sockets'].values()))
        self.assertIsNone(row['sk_drops_delta'])
        (self.root / 'net/udp').unlink()
        unavailable = self.sampler._udp_sockets(1)
        self.assertEqual(unavailable['status'], 'partial')
        self.assertEqual(unavailable['sockets'], {})
        self.assertEqual(unavailable['ports_without_visible_socket'], [2368])

    def test_optional_process_cpu_and_pid_reuse(self):
        self.assertIsNone(self.sampler._processes(None)['42']['cpu_percent'])
        (self.root / '42/stat').write_text(process_fixture(cpu=200))
        process = self.sampler._processes(1)['42']
        self.assertAlmostEqual(process['cpu_percent'], 10000 / self.sampler.clock_ticks)
        self.assertEqual(process['rss_bytes'], 10 * self.sampler.page_size)
        (self.root / '42/stat').write_text(process_fixture(cpu=200, start=99))
        process = self.sampler._processes(1)['42']
        self.assertTrue(process['pid_reused'])
        self.assertIsNone(process['cpu_percent'])
        (self.root / '42/stat').unlink()
        self.assertEqual(self.sampler._processes(1)['42']['status'], 'unavailable')

    def test_permission_failure_does_not_claim_zero_or_reuse_short_interval(self):
        self.sampler._network(None)
        self.sampler._udp(None)
        with patch.object(self.sampler, '_read', side_effect=PermissionError(13, 'forbidden')):
            self.assertEqual(self.sampler._network(1)['error_type'], 'PermissionError')
            self.assertEqual(self.sampler._udp(1)['status'], 'unavailable')
            self.assertEqual(self.sampler._cpu(1)['status'], 'unavailable')
            self.assertEqual(self.sampler._memory()['status'], 'unavailable')
        # An unread interval must not turn two intervals of traffic into one interval's rate.
        self.assertIsNone(self.sampler._network(1)['interfaces']['eth0']['per_second']['rx_bytes'])
        self.assertIsNone(self.sampler._udp(1)['per_second']['InDatagrams'])

    def test_no_nvidia_is_explicit_and_does_not_spawn(self):
        with patch.object(telemetry.shutil, 'which', return_value=None), patch.object(telemetry.subprocess, 'run') as run:
            sampler = telemetry.HostSampler(proc_root=self.root)
            self.assertEqual(sampler._gpu(1)['error_type'], 'NvidiaSmiNotFound')
            run.assert_not_called()

    def test_gpu_timeout_and_unsupported_fields(self):
        with patch.object(telemetry.shutil, 'which', return_value='/usr/bin/nvidia-smi'):
            sampler = telemetry.HostSampler(proc_root=self.root)
        with patch.object(telemetry.subprocess, 'run', side_effect=subprocess.TimeoutExpired('nvidia-smi', 1)):
            self.assertEqual(sampler._gpu(1)['error_type'], 'TimeoutExpired')
        answer = subprocess.CompletedProcess([], 0, '0, GPU-test, 40, N/A, 100, 200, 50, [Not Supported]\n')
        with patch.object(telemetry.subprocess, 'run', return_value=answer) as run:
            first = sampler._gpu(6_000_000_000)
            self.assertEqual(first['gpus'][0]['utilization.gpu'], 40)
            self.assertIsNone(first['gpus'][0]['power.draw'])
            self.assertIsNone(first['gpus'][0]['utilization.memory'])
            self.assertEqual(sampler._gpu(7_000_000_000)['age_sec'], 1)
            self.assertEqual(run.call_count, 1)
            self.assertEqual(run.call_args.kwargs['timeout'], 1)

    def test_sample_and_provenance_are_json_safe(self):
        sample = self.sampler.sample()
        self.assertEqual(sample['session_id'], 'test-session')
        self.assertEqual(sample['gpu']['status'], 'disabled')
        self.assertGreater(sample['wall_time_ns'], 0)
        self.assertGreater(sample['monotonic_ns'], 0)
        self.assertIsNone(sample['elapsed_sec'])
        self.assertEqual(sample['memory']['MemAvailable_bytes'], 400 * 1024)
        self.assertEqual(sample['load']['runnable_tasks'], 1)
        metadata = self.sampler.provenance()
        self.assertFalse(metadata['clocks']['cross_host_synchronization_verified'])
        self.assertEqual(metadata['session_id'], 'test-session')
        json.dumps([sample, metadata], allow_nan=False)

    def test_cli_finite_duration_and_existing_file_protection(self):
        output = self.root / 'host.jsonl'
        self.assertEqual(telemetry.main(['--output', str(output), '--duration', '.41', '--interval', '.2',
                                         '--no-gpu', '--session-id', 'abc']), 0)
        records = [json.loads(line) for line in output.read_text().splitlines()]
        self.assertEqual(records[0]['record_type'], 'metadata')
        self.assertEqual(records[-1]['record_type'], 'end')
        self.assertGreaterEqual(records[-1]['samples'], 1)
        self.assertLessEqual(records[-1]['samples'], 3)
        self.assertTrue(all(record['session_id'] == 'abc' for record in records))
        before = output.read_bytes()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            telemetry.main(['--output', str(output), '--no-gpu'])
        self.assertEqual(output.read_bytes(), before)

    def test_cli_sigterm_flushes_end_record_without_killing_other_processes(self):
        output = self.root / 'terminated.jsonl'
        process = subprocess.Popen([sys.executable, str(Path(__file__).with_name('record_host_telemetry.py')),
                                    '--output', str(output), '--no-gpu', '--interval', '.2'],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                # A sample is written only after signal handlers are installed.
                if output.exists() and '"record_type": "sample"' in output.read_text():
                    break
                if process.poll() is not None:
                    self.fail('collector ended unexpectedly')
                time.sleep(.01)
            else:
                self.fail('collector never produced a sample')
            process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 0, stderr)
            records = [json.loads(line) for line in output.read_text().splitlines()]
            self.assertEqual(records[-1]['record_type'], 'end')
            self.assertEqual(records[-1]['signal'], signal.SIGTERM)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()

    def test_bad_cli_options_fail_without_creating_file(self):
        output = self.root / 'invalid.jsonl'
        for options in (['--interval', 'nan'], ['--duration', '-1'], ['--gpu-timeout', '2'],
                        ['--udp-port', '0'], ['--pid', '-4'], ['--interfaces', ' , ']):
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                telemetry.main(['--output', str(output)] + options)
        self.assertFalse(output.exists())

    def test_malformed_input_is_explicit(self):
        (self.root / 'stat').write_text('cpu invalid\n')
        (self.root / 'net/snmp').write_text('Ip: fields\nIp: 0\n')
        self.assertEqual(self.sampler._cpu(1)['status'], 'unavailable')
        self.assertEqual(self.sampler._udp(1)['status'], 'unavailable')
        self.assertEqual(self.sampler._udp_sockets(1)['sources']['ipv4']['status'], 'ok')


if __name__ == '__main__':
    unittest.main()
