#!/usr/bin/env python3
"""Optional, narrowly allowlisted local backing-file evidence; not memory attestation."""
import argparse
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import socket
import stat
import subprocess
import sys
from urllib.parse import urlsplit
import xmlrpc.client

TARGETS = (
    ('mpt', '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer',
     ('path_optimizer_node',), 'libautoware_path_optimizer.so'),
    ('traffic', '/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner',
     ('behavior_velocity_planner_full_node', 'behavior_velocity_planner_full_experimental_node'),
     'libautoware_behavior_velocity_traffic_light_module.so'),
)
MAX_FILE_BYTES = 64 * 1024 * 1024
MAX_MAP_BYTES = 2 * 1024 * 1024
CALLER = '/development_bag_recorder'


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 0.5
        if connection.sock is not None:
            connection.sock.settimeout(0.5)
        return connection


def proxy(uri):
    return xmlrpc.client.ServerProxy(uri, transport=Transport(), allow_none=True)


def local_address(host):
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        return False
    if address.is_unspecified:
        return False
    if address.is_loopback:
        return True
    family = socket.AF_INET6 if address.version == 6 else socket.AF_INET
    try:
        with socket.socket(family, socket.SOCK_STREAM) as probe:
            probe.bind((str(address), 0))
        return True
    except OSError:
        return False


def local_host(host):
    """Names alone are not authority; every resolved address must be local.

    DNS, including a broken hostname mapping, is bounded by the parent collector
    timeout. Neither hostnames nor resolved addresses are saved in the receipt.
    """
    if not host:
        return False
    try:
        ipaddress.ip_address(host)
        return local_address(host)
    except ValueError:
        try:
            addresses = {entry[4][0] for entry in socket.getaddrinfo(
                host, None, family=socket.AF_UNSPEC, type=socket.SOCK_STREAM)}
            return bool(addresses) and all(local_address(address) for address in addresses)
        except OSError:
            return False


def valid_uri(uri):
    parsed = urlsplit(uri)
    if parsed.scheme != 'http' or parsed.username or parsed.password or not parsed.hostname:
        raise ValueError('Unsupported ROS URI')
    return parsed


def rpc_result(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3 or value[0] != 1:
        raise LookupError('ROS node unavailable')
    return value[2]


def start_ticks(proc):
    # Only field22 is retained; no command line, environment or full stat text.
    text = (proc / 'stat').read_text()[:8192]
    return int(text[text.rindex(')') + 1:].split()[19])


def identity(value):
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns)


def disk_receipt(path, device=None, inode=None):
    result = {'path': str(path)}
    before = path.stat()
    result.update(device=before.st_dev, inode=before.st_ino, size=before.st_size,
                  mtime_ns=before.st_mtime_ns)
    if not stat.S_ISREG(before.st_mode):
        return dict(result, status='not_regular_file')
    if before.st_size > MAX_FILE_BYTES:
        return dict(result, status='too_large')
    if ((device is not None and before.st_dev != device) or
            (inode is not None and before.st_ino != inode)):
        return dict(result, status='inode_mismatch')
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        if identity(os.fstat(handle.fileno())) != identity(before):
            return dict(result, status='file_changed')
        prefix = handle.read(4)
        if prefix != b'\x7fELF':
            return dict(result, status='not_elf')
        digest.update(prefix)
        consumed = 4
        while True:
            chunk = handle.read(1024 * 1024)
            if not chunk:
                break
            consumed += len(chunk)
            if consumed > MAX_FILE_BYTES:
                return dict(result, status='too_large')
            digest.update(chunk)
        after = os.fstat(handle.fileno())
    if identity(after) != identity(before) or identity(path.stat()) != identity(before):
        return dict(result, status='file_changed')
    return dict(result, status='identity_verified_disk_file', sha256=digest.hexdigest(),
                sha256_kind='backing_disk_file_not_process_memory')


def target_receipt(master, node, executables, library, proc_root, proxy_factory):
    node_uri = rpc_result(master.lookupNode(CALLER, node))
    parsed = valid_uri(node_uri)
    if not local_host(parsed.hostname):
        return {'status': 'remote_or_unverified_host'}
    pid = rpc_result(proxy_factory(node_uri).getPid(CALLER))
    if type(pid) is not int or pid <= 0:
        return {'status': 'invalid_pid'}
    proc = proc_root / str(pid)
    initial_ticks = start_ticks(proc)
    exe_link = os.readlink(proc / 'exe')
    base = {'pid': pid, 'start_ticks': initial_ticks, 'local_host_verified': True}
    if exe_link.endswith(' (deleted)'):
        return dict(base, status='deleted_executable')
    if Path(exe_link).name not in executables:
        return dict(base, status='unexpected_executable')
    exe_stat = (proc / 'exe').stat()
    exe = disk_receipt(Path(exe_link), exe_stat.st_dev, exe_stat.st_ino)
    matches = {}
    consumed = 0
    # Read maps internally but retain ONLY the selected exact library entry.
    with (proc / 'maps').open('rb') as handle:
        for line in handle:
            consumed += len(line)
            if consumed > MAX_MAP_BYTES:
                return dict(base, status='maps_too_large')
            parts = line.decode('utf-8', errors='replace').rstrip('\n').split(None, 5)
            if len(parts) != 6:
                continue
            name = parts[5]
            deleted = name.endswith(' (deleted)')
            plain = name[:-10] if deleted else name
            if Path(plain).name != library:
                continue
            major, minor = parts[3].split(':')
            device = os.makedev(int(major, 16), int(minor, 16))
            inode = int(parts[4])
            matches[(plain, device, inode, deleted)] = None
    if len(matches) > 1:
        mapped = {'status': 'ambiguous_library'}
    elif not matches:
        mapped = {'status': 'not_loaded'}
    else:
        (name, device, inode, deleted), = matches
        mapped = ({'path': name, 'status': 'deleted_mapping'} if deleted else
                  disk_receipt(Path(name), device, inode))
    if (start_ticks(proc) != initial_ticks or os.readlink(proc / 'exe') != exe_link or
            identity((proc / 'exe').stat()) != identity(exe_stat)):
        return dict(base, status='process_identity_changed')
    return dict(base, status='recorded', executable=exe, library=mapped)


def collect(master_uri, proc_root=Path('/proc'), proxy_factory=proxy):
    result = {'status': 'completed', 'targets': {},
              'limits': 'Allowlisted local backing disk files only; not memory bytes, '
                        'active code branches, map, model or GPU-backend attestation. '
                        'Mapping presence is a snapshot, not an atomic lifetime guarantee.'}
    try:
        valid_uri(master_uri)
        master = proxy_factory(master_uri)
    except Exception as error:
        return dict(result, status='master_unavailable', error_type=type(error).__name__)
    for key, node, executables, library in TARGETS:
        try:
            entry = target_receipt(master, node, executables, library, proc_root, proxy_factory)
        except PermissionError:
            entry = {'status': 'permission_denied'}
        except (TimeoutError, socket.timeout):
            entry = {'status': 'rpc_timeout'}
        except LookupError:
            entry = {'status': 'node_missing'}
        except Exception as error:
            # Never serialize raw RPC URLs, arbitrary filenames or exception text.
            entry = {'status': 'unavailable', 'error_type': type(error).__name__}
        result['targets'][key] = dict(entry, node=node)
    return result


def collect_bounded(master_uri, timeout=3.0):
    if not master_uri:
        return {'status': 'master_uri_unavailable', 'targets': {}}
    try:
        # Reject userinfo before it could appear in the collector command line.
        valid_uri(master_uri)
        completed = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), '--master-uri', master_uri],
            capture_output=True, text=True, timeout=timeout, check=False)
        if completed.returncode:
            return {'status': 'collector_failed', 'targets': {}}
        result = json.loads(completed.stdout)
        if not isinstance(result, dict) or not isinstance(result.get('targets'), dict):
            raise ValueError('Malformed collector result')
        return result
    except subprocess.TimeoutExpired:
        return {'status': 'collector_timeout', 'targets': {}}
    except Exception as error:
        return {'status': 'collector_unavailable', 'targets': {}, 'error_type': type(error).__name__}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master-uri', required=True)
    args = parser.parse_args()
    print(json.dumps(collect(args.master_uri), ensure_ascii=False))
