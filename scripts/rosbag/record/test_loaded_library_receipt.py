"""Fake RPC/proc fixtures only: never query a live ROS master or stack PID."""
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('loaded_receipt',
                                             Path(__file__).with_name('loaded_library_receipt.py'))
receipt = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(receipt)


class LoadedReceiptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='loaded-receipt-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.proc = self.root / 'proc'
        self.nodes = {}
        self.pids = {}
        for index, (key, node, exe, library) in enumerate(receipt.TARGETS):
            pid = 41 + index
            directory = self.proc / str(pid)
            directory.mkdir(parents=True)
            binary = self.root / exe
            binary.write_bytes(b'\x7fELF' + key.encode())
            (directory / 'exe').symlink_to(binary)
            # Fields3..21 precede starttime field22.
            (directory / 'stat').write_text(str(pid) + ' (name with spaces) S ' + '0 ' * 18 + '123 0\n')
            shared = self.root / library
            shared.write_bytes(b'\x7fELF' + library.encode())
            self.write_maps(directory, shared)
            uri = 'http://127.0.0.1:' + str(15000 + index)
            self.nodes[node] = uri
            self.pids[uri] = pid
        self.master_uri = 'http://fake-master.invalid:11311'

    def write_maps(self, directory, library, suffix='', inode=None):
        info = library.stat()
        device = '{:x}:{:x}'.format(os.major(info.st_dev), os.minor(info.st_dev))
        line = '1000-2000 r-xp 0000 {} {} {}{}\n'.format(
            device, info.st_ino if inode is None else inode, library, suffix)
        # Arbitrary mappings must never leak into evidence.
        (directory / 'maps').write_text(line + '3000-4000 rw-p 0000 00:00 0 /private/TOP_SECRET_TOKEN\n')

    def factory(self, uri):
        if uri == self.master_uri:
            def lookup(caller, node):
                return [1, '', self.nodes[node]] if node in self.nodes else [-1, '', None]
            return types.SimpleNamespace(lookupNode=lookup)
        return types.SimpleNamespace(getPid=lambda caller: [1, '', self.pids[uri]])

    def collect(self):
        return receipt.collect(self.master_uri, self.proc, self.factory)

    def test_success_contains_only_allowlisted_disk_identity_and_hashes(self):
        result = self.collect()
        for key in ('mpt', 'traffic'):
            target = result['targets'][key]
            self.assertEqual(target['status'], 'recorded')
            self.assertEqual(target['start_ticks'], 123)
            for field in ('executable', 'library'):
                self.assertEqual(target[field]['status'], 'identity_verified_disk_file')
                self.assertEqual(len(target[field]['sha256']), 64)
                self.assertEqual(target[field]['sha256_kind'], 'backing_disk_file_not_process_memory')
        text = json.dumps(result)
        self.assertNotIn('TOP_SECRET', text)
        self.assertNotIn('cmdline', text)
        self.assertNotIn('environ', text)

    def test_remote_or_unverified_node_never_queries_pid_or_local_proc(self):
        self.nodes[receipt.TARGETS[0][1]] = 'http://198.51.100.9:1234'
        with patch.object(receipt, 'local_host', return_value=False), \
                patch.object(receipt, 'start_ticks', side_effect=AssertionError('must not read proc')):
            result = self.collect()
        self.assertEqual(result['targets']['mpt']['status'], 'remote_or_unverified_host')

    def test_deleted_mapping_is_not_given_a_current_disk_hash(self):
        self.write_maps(self.proc / '41', self.root / receipt.TARGETS[0][3], suffix=' (deleted)')
        library = self.collect()['targets']['mpt']['library']
        self.assertEqual(library['status'], 'deleted_mapping')
        self.assertNotIn('sha256', library)

    def test_inode_mismatch_is_not_called_loaded_file_hash(self):
        self.write_maps(self.proc / '41', self.root / receipt.TARGETS[0][3], inode=1)
        library = self.collect()['targets']['mpt']['library']
        self.assertEqual(library['status'], 'inode_mismatch')
        self.assertNotIn('sha256', library)

    def test_pid_reuse_discards_previously_computed_file_hashes(self):
        with patch.object(receipt, 'start_ticks', side_effect=[123, 124, 123, 123]):
            target = self.collect()['targets']['mpt']
        self.assertEqual(target['status'], 'process_identity_changed')
        self.assertNotIn('executable', target)
        self.assertNotIn('library', target)

    def test_missing_and_permission_failures_are_optional_without_error_text(self):
        self.nodes.pop(receipt.TARGETS[0][1])
        self.assertEqual(self.collect()['targets']['mpt']['status'], 'node_missing')
        with patch.object(receipt, 'target_receipt', side_effect=PermissionError('SECRET')):
            result = self.collect()
        self.assertEqual(result['targets']['mpt']['status'], 'permission_denied')
        self.assertNotIn('SECRET', json.dumps(result))

    def test_unexpected_executable_is_not_hashed(self):
        exe = self.proc / '41' / 'exe'
        exe.unlink()
        wrong = self.root / 'not_allowlisted'
        wrong.write_bytes(b'credentials')
        exe.symlink_to(wrong)
        self.assertEqual(self.collect()['targets']['mpt']['status'], 'unexpected_executable')

    def test_large_maps_and_large_files_fail_without_hashes(self):
        with patch.object(receipt, 'MAX_MAP_BYTES', 1):
            self.assertEqual(self.collect()['targets']['mpt']['status'], 'maps_too_large')
        with patch.object(receipt, 'MAX_FILE_BYTES', 1):
            target = self.collect()['targets']['mpt']
            self.assertEqual(target['library']['status'], 'too_large')
            self.assertNotIn('sha256', target['library'])

    def test_rpc_timeout_is_optional_and_does_not_expose_rpc_secrets(self):
        with patch.object(receipt, 'target_receipt', side_effect=socket.timeout('SECRET')):
            result = self.collect()
        self.assertEqual(result['targets']['mpt']['status'], 'rpc_timeout')
        self.assertNotIn('SECRET', json.dumps(result))

    def test_collector_hang_and_failure_are_bounded_nonblocking_statuses(self):
        with patch.object(receipt.subprocess, 'run', side_effect=subprocess.TimeoutExpired('SECRET', 3)) as run:
            result = receipt.collect_bounded(self.master_uri)
        self.assertEqual(result['status'], 'collector_timeout')
        self.assertEqual(run.call_args.kwargs['timeout'], 3.0)
        self.assertNotIn('SECRET', json.dumps(result))
        with patch.object(receipt.subprocess, 'run', return_value=types.SimpleNamespace(returncode=1)):
            self.assertEqual(receipt.collect_bounded(self.master_uri)['status'], 'collector_failed')

    def test_no_master_uri_never_starts_collector(self):
        with patch.object(receipt.subprocess, 'run') as run:
            self.assertEqual(receipt.collect_bounded(None)['status'], 'master_uri_unavailable')
        run.assert_not_called()

    def test_master_credentials_are_rejected_before_child_command_line(self):
        with patch.object(receipt.subprocess, 'run') as run:
            result = receipt.collect_bounded('http://user:TOP_SECRET@127.0.0.1:11311')
        run.assert_not_called()
        self.assertEqual(result['status'], 'collector_unavailable')
        self.assertNotIn('TOP_SECRET', json.dumps(result))

    def test_credentials_in_ros_uri_are_not_collected(self):
        self.nodes[receipt.TARGETS[0][1]] = 'http://user:TOP_SECRET@127.0.0.1:1234'
        result = self.collect()
        self.assertEqual(result['targets']['mpt']['status'], 'unavailable')
        self.assertNotIn('TOP_SECRET', json.dumps(result))

    def test_hostname_string_is_not_local_authority_when_resolution_is_remote(self):
        resolved = [(socket.AF_INET, socket.SOCK_STREAM, 6, '', ('198.51.100.9', 0))]
        with patch.object(socket, 'getaddrinfo', return_value=resolved), \
                patch.object(receipt, 'local_address', return_value=False):
            self.assertFalse(receipt.local_host(socket.gethostname()))

    def test_file_mutation_discards_hash(self):
        path = self.root / receipt.TARGETS[0][3]
        before = path.stat()
        changed = types.SimpleNamespace(st_dev=before.st_dev, st_ino=before.st_ino,
                                        st_size=before.st_size, st_mtime_ns=before.st_mtime_ns + 1,
                                        st_ctime_ns=before.st_ctime_ns)
        with patch.object(receipt.os, 'fstat', side_effect=[before, changed]):
            result = receipt.disk_receipt(path)
        self.assertEqual(result['status'], 'file_changed')
        self.assertNotIn('sha256', result)

    def test_parent_timeout_really_reaps_its_own_hanging_collector_only(self):
        real_run = subprocess.run
        def hanging_collector(*args, **kwargs):
            return real_run([sys.executable, '-c', 'import time;time.sleep(10)'],
                            capture_output=True, text=True, timeout=0.05)
        started = time.monotonic()
        with patch.object(receipt.subprocess, 'run', side_effect=hanging_collector):
            result = receipt.collect_bounded(self.master_uri)
        self.assertEqual(result['status'], 'collector_timeout')
        self.assertLess(time.monotonic() - started, 2.0)


if __name__ == '__main__':
    unittest.main()
