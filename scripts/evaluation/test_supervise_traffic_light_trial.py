"""Pure mocked tests: never load ROS, start processes or send real signals."""
import json
from pathlib import Path
import signal
from types import SimpleNamespace

import pytest

import supervise_traffic_light_trial as supervisor


class FakeProcess:
    def __init__(self, pid, created=1., parent=10, command=(), role=None):
        self.pid, self.created, self.parent = pid, created, parent
        self.command, self.running, self.descendants, self.signals = command, True, [], []
    def create_time(self): return self.created
    def ppid(self): return self.parent
    def cmdline(self): return self.command
    def is_running(self): return self.running
    def status(self): return 'running' if self.running else 'zombie'
    def children(self, recursive=False): return self.descendants
    def send_signal(self, number):
        self.signals.append(number)
        self.running = False


def ownership():
    owner = FakeProcess(10)
    controller = FakeProcess(20, command=['python3', 'roslaunch', 'trajectory_follower', 'pp_pi_autoware.launch'])
    node = FakeProcess(21, parent=20, command=['/some/controller_binary'])
    udp = FakeProcess(30, command=['roslaunch', 'sensor_interface', 'morai_udp_interface.launch', 'enable_control:=true'])
    sensor = FakeProcess(40, command=['roslaunch', 'sensor_interface', 'morai_udp_interface.launch', 'enable_control:=false'])
    owner.descendants = [node, udp, sensor, controller]
    processes = {row.pid: row for row in [owner, controller, node, udp, sensor]}
    events = []
    owned = supervisor.OwnedProcesses(10, processes.__getitem__, lambda event, **kw: events.append((event, kw)))
    owned.capture()
    return owned, processes, events


def test_owner_snapshot_uses_exact_command_and_unordered_ancestry_only():
    owned, processes, _ = ownership()
    assert {row['pid'] for row in owned.active({'controller'})} == {20, 21}
    assert {row['pid'] for row in owned.active({'udp_control'})} == {30}
    processes[10].running = False
    outsider = FakeProcess(99, command=processes[20].command)
    processes[99] = outsider
    owned.capture()
    assert 99 not in {row['pid'] for row in owned.active()}
    owned.stop({'controller'})
    assert processes[20].signals == [signal.SIGINT]
    assert processes[21].signals == [signal.SIGINT]
    assert not processes[30].signals and not outsider.signals


def test_pid_reuse_is_never_signalled_even_with_same_controller_command():
    owned, processes, _ = ownership()
    replacement = FakeProcess(20, created=2., command=processes[20].command)
    processes[20] = replacement
    owned.stop({'controller'})
    assert not replacement.signals


def test_reparenting_does_not_erase_already_proven_controller_role():
    owned, processes, _ = ownership()
    processes[21].parent = 1359
    owned.capture()
    assert 21 in {row['pid'] for row in owned.active({'controller'})}


def test_supervisor_error_interrupt_targets_identity_checked_owner_not_process_group():
    owned, processes, _ = ownership()
    owner = processes[10]
    owner.poll = lambda: None if owner.running else -signal.SIGINT
    supervisor.stop_owner_for_recovery(owner, owned, lambda *a, **k: None)
    assert owner.signals == [signal.SIGINT]
    assert not processes[20].signals  # Recovery, not a broad group signal, owns this later.


def test_arbitrary_command_missing_profile_and_existing_output_rejected(tmp_path):
    network = tmp_path/'network.json'
    network.write_text('{}')
    out = supervisor.ROOT/'artifacts/evaluation/supervisor_test_not_created'
    with pytest.raises(ValueError):
        supervisor.validate_request(['/usr/bin/python3', '/tmp/not_the_wrapper.py', '--help'], network, out)
    with pytest.raises(FileNotFoundError):
        supervisor.validate_request(['/usr/bin/python3', str(supervisor.WRAPPER), '--help'], tmp_path/'missing', out)
    with pytest.raises(FileExistsError):
        supervisor.validate_request(['/usr/bin/python3', str(supervisor.WRAPPER), '--help'], network, supervisor.ROOT/'artifacts/evaluation')


class FakeOwned:
    def __init__(self, controls): self.controls, self.calls = controls, []
    def active(self, roles=None): return self.controls
    def stop(self, roles=None, grace=12.):
        self.calls.append(('stop', roles))
        return []


def test_normal_exit_without_controls_never_initializes_ros_or_mutates_processes(tmp_path):
    owned = FakeOwned([])
    def forbidden(): raise AssertionError('ROS must not be initialized')
    result = supervisor.finish_trial(0, owned, tmp_path/'network', tmp_path, lambda *a, **k: None, forbidden)
    assert result['status'] == 'NORMAL' and not owned.calls


def test_failure_without_control_survivors_does_not_publish_brake_or_park(tmp_path):
    owned = FakeOwned([])
    def forbidden(): raise AssertionError('ROS must not be initialized')
    result = supervisor.finish_trial(143, owned, tmp_path/'network', tmp_path, lambda *a, **k: None, forbidden)
    assert result['status'] == 'OWNER_FAILED_NO_CONTROL_SURVIVORS'
    assert owned.calls == [('stop', None)]


def fake_runtime(events, fail_stop=False):
    class Monitor:
        def verify_stopped(self, **kwargs):
            events.append(('verify', kwargs))
            if fail_stop: raise RuntimeError('no fresh inputs')
            return dict(park=kwargs.get('require_park', False))
    class Hold:
        def start(self): events.append(('brake_start',))
        def stop(self): events.append(('brake_stop',))
    class Processes:
        def __init__(self, *args): pass
        def start(self, name, command): events.append(('park_start', command))
        def close(self): events.append(('park_close',))
    return SimpleNamespace(LiveMonitor=Monitor, BrakeHold=Hold, Processes=Processes)


def test_owner_failure_recovers_stop_then_single_sender_then_fresh_P(tmp_path):
    events, owned = [], FakeOwned([dict(pid=20)])
    original_stop = owned.stop
    def stop(roles=None, grace=12.):
        events.append(('owned_stop', roles))
        return original_stop(roles, grace)
    owned.stop = stop
    result = supervisor.finish_trial(143, owned, tmp_path/'same_network.json', tmp_path,
        lambda *a, **k: None, lambda: fake_runtime(events), clock=lambda: 50.)
    assert result['status'] == 'RECOVERED_STOP_P_VERIFIED'
    assert [row[0] for row in events] == ['owned_stop', 'brake_start', 'verify',
        'owned_stop', 'park_start', 'verify', 'park_close', 'brake_stop', 'owned_stop']
    assert events[2][1]['after'] == 50. and events[2][1]['freshness_sec'] == 1.
    assert events[5][1]['require_park'] is True
    assert events[4][1][-1] == 'network_config:='+str(tmp_path/'same_network.json')


def test_missing_fresh_stop_never_claims_P_or_starts_parking_sender(tmp_path):
    events, owned = [], FakeOwned([dict(pid=30)])
    result = supervisor.finish_trial(143, owned, tmp_path/'network', tmp_path,
        lambda *a, **k: None, lambda: fake_runtime(events, fail_stop=True), clock=lambda: 50.)
    assert result['status'] == 'UNVERIFIED' and 'park' not in result
    assert not any(row[0] == 'park_start' for row in events)
    assert ('brake_stop',) in events and owned.calls[-1] == ('stop', None)


def test_successful_owner_exit_with_leftover_controller_still_requires_recovery(tmp_path):
    events, owned = [], FakeOwned([dict(pid=20)])
    result = supervisor.finish_trial(0, owned, tmp_path/'network', tmp_path,
        lambda *a, **k: None, lambda: fake_runtime(events), clock=lambda: 50.)
    assert result['recovery_attempted'] and result['status'] == 'RECOVERED_STOP_P_VERIFIED'
