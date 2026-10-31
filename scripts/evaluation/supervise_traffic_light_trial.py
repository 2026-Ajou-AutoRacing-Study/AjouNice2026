#!/usr/bin/env python3
"""Eval-only owner supervisor; never controls lamps or guesses orphan ownership.

Run from an independent tmux session with the ROS environment already sourced.
Only the exact traffic-light trial wrapper may be launched. The supervisor
snapshots descendant PID/create-time identities while the owner is alive. On
owner failure, only those identities can be signalled or recovered.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
WRAPPER = ROOT / 'scripts/evaluation/run_traffic_light_compliance_trial.py'
RUNTIME_ROOT = ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/src'
PARK_LAUNCH = ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/launch/parking_hold.launch'


def validate_request(command, network_config, output):
    command = list(command)
    if command[:1] == ['--']:
        command.pop(0)
    if len(command) < 3:
        raise ValueError('Use -- /absolute/python3 exact_trial_wrapper.py [trial arguments]')
    python = Path(command[0]).expanduser()
    if (not python.is_absolute() or not python.is_file() or not os.access(python, os.X_OK)
            or not python.name.startswith('python')):
        raise ValueError('An explicit existing Python executable is required')
    if Path(command[1]).expanduser().resolve() != WRAPPER.resolve() or not WRAPPER.is_file():
        raise ValueError('Only the exact repository traffic-light trial wrapper is allowed')
    network = Path(network_config).expanduser().resolve(strict=True)
    if not isinstance(json.loads(network.read_text()), dict):
        raise ValueError('An explicit existing JSON network profile is required')
    output = Path(output).expanduser().resolve()
    output.relative_to(ROOT / 'artifacts/evaluation')
    if output.exists():
        raise FileExistsError('Fresh supervisor output required')
    command[0], command[1] = str(python), str(WRAPPER)
    return command, network, output


def direct_role(command):
    """Only exact known roslaunch entries qualify as control ownership roots."""
    names = [Path(value).name for value in command]
    if 'roslaunch' not in names:
        return None
    if 'pp_pi_autoware.launch' in names and 'trajectory_follower' in command:
        return 'controller'
    if ('morai_udp_interface.launch' in names and 'sensor_interface' in command
            and 'enable_control:=true' in command):
        return 'udp_control'
    return None


class OwnedProcesses:
    def __init__(self, owner_pid, process_factory, emit, clock=time.monotonic, wait=time.sleep):
        self.owner_pid, self.process_factory, self.emit = owner_pid, process_factory, emit
        self.clock, self.wait, self.records = clock, wait, {}
        owner = process_factory(owner_pid)
        self.owner_created = owner.create_time()

    def same_process(self, record):
        try:
            process = self.process_factory(record['pid'])
            if (process.create_time() != record['create_time'] or not process.is_running()
                    or process.status() == 'zombie'):
                return None
            return process
        except (ProcessLookupError, OSError):
            return None
        except Exception as error:
            # psutil NoSuchProcess/AccessDenied are deliberately not authority.
            self.emit('identity_unavailable', pid=record['pid'], error=type(error).__name__)
            return None

    def capture(self):
        owner_record = dict(pid=self.owner_pid, create_time=self.owner_created)
        owner = self.same_process(owner_record)
        if owner is None:
            return
        try:
            descendants = owner.children(recursive=True)
        except Exception as error:
            self.emit('ownership_capture_failed', error=type(error).__name__)
            return
        current = {}
        for process in descendants:
            try:
                row = dict(pid=process.pid, create_time=process.create_time(),
                           ppid=process.ppid(), command=process.cmdline(), role=None)
                current[row['pid']] = row
            except Exception:
                continue
        # Descendant order is not an API contract. Resolve roles via ancestry.
        for row in current.values():
            role, parent, seen = direct_role(row['command']), row['ppid'], set()
            while role is None and parent in current and parent not in seen:
                seen.add(parent)
                role = direct_role(current[parent]['command'])
                parent = current[parent]['ppid']
            row['role'] = role
            key = (row['pid'], row['create_time'])
            previous = self.records.get(key)
            # Reparenting must not erase an already proven controller ancestry.
            if row['role'] is None and previous is not None:
                row['role'] = previous['role']
            if previous is None or row['role'] != previous['role']:
                self.emit('owned_identity', **row)
            self.records[key] = row

    def active(self, roles=None):
        return [row for row in self.records.values()
                if (roles is None or row['role'] in roles) and self.same_process(row) is not None]

    def stop(self, roles=None, grace=12.):
        """Broadcast bounded shutdown only to snapshot identities, no killpg."""
        selected = self.active(roles)
        for number, seconds in ((signal.SIGINT, grace), (signal.SIGTERM, 4.), (signal.SIGKILL, 2.)):
            for row in selected:
                process = self.same_process(row)  # Re-check immediately before EVERY signal.
                if process is not None:
                    try:
                        process.send_signal(number)
                        self.emit('owned_signal', pid=row['pid'], create_time=row['create_time'],
                                  role=row['role'], signal=int(number))
                    except Exception as error:
                        self.emit('owned_signal_failed', pid=row['pid'], error=type(error).__name__)
            until = self.clock() + seconds
            while self.clock() < until:
                selected = [row for row in selected if self.same_process(row) is not None]
                if not selected:
                    return []
                self.wait(min(.05, max(0., until-self.clock())))
        remaining = [row for row in selected if self.same_process(row) is not None]
        if remaining:
            self.emit('owned_shutdown_unverified', identities=remaining)
        return remaining


def real_runtime():
    sys.path.insert(0, str(RUNTIME_ROOT))
    import rospy
    from planning_scenario_validator import runtime
    rospy.init_node('traffic_trial_orphan_supervisor', anonymous=True, disable_signals=True)
    return runtime


def stop_owner_for_recovery(process, owned, emit, clock=time.monotonic, wait=time.sleep):
    """Bound even an unexpected supervisor exception before orphan recovery."""
    row = dict(pid=process.pid, create_time=owned.owner_created)
    owner = owned.same_process(row)
    if owner is not None:
        owner.send_signal(signal.SIGINT)
        emit('owner_error_interrupt_forwarded')
    until = clock()+5.
    while process.poll() is None and clock() < until:
        owned.capture()
        wait(.1)
    owner = owned.same_process(row)
    if process.poll() is None and owner is not None:
        owner.send_signal(signal.SIGKILL)
        process.wait(timeout=3.)


def finish_trial(returncode, owned, network, output, emit, runtime_factory=real_runtime,
                 clock=time.monotonic):
    controls = owned.active({'controller', 'udp_control'})
    if returncode == 0 and not controls:
        emit('normal_owner_exit_no_control_survivors', returncode=returncode)
        return dict(status='NORMAL', owner_returncode=returncode, recovery_attempted=False)
    if not controls:
        remaining = owned.stop(grace=30.)
        return dict(status='OWNER_FAILED_NO_CONTROL_SURVIVORS', owner_returncode=returncode,
                    recovery_attempted=False, remaining_identities=remaining)
    emit('orphan_recovery_required', owner_returncode=returncode, identities=controls)
    hold = parking = None
    result = dict(status='UNVERIFIED', owner_returncode=returncode, recovery_attempted=True)
    try:
        remaining = owned.stop({'controller'}, grace=5.)
        if remaining:
            raise RuntimeError('Owned controller shutdown not verified; no competing brake publisher started')
        runtime = runtime_factory()
        monitor, hold = runtime.LiveMonitor(), runtime.BrakeHold()
        hold.start()
        result['stop'] = monitor.verify_stopped(after=clock(), timeout=15., hold_sec=1.,
            freshness_sec=1., speed_max_mps=.1)
        emit('orphan_stop_verified', receipt=result['stop'])
        if owned.stop({'udp_control'}, grace=5.):
            raise RuntimeError('Owned UDP shutdown not verified; refusing a second sender')
        parking = runtime.Processes(output / 'parking_logs', os.environ.copy())
        after = clock()
        parking.start('park', ['roslaunch', str(PARK_LAUNCH), 'network_config:='+str(network)])
        result['park'] = monitor.verify_stopped(after=after, timeout=15., hold_sec=1.,
            freshness_sec=1., speed_max_mps=.1, require_park=True)
        result['status'] = 'RECOVERED_STOP_P_VERIFIED'
        emit('orphan_park_verified', receipt=result['park'])
    except Exception as error:
        result['error'] = type(error).__name__+': '+str(error)
        emit('orphan_recovery_unverified', error=result['error'])
    finally:
        # Even absent sensors/communications must not be called successful P.
        if parking is not None:
            try:
                parking.close()
            except Exception as error:
                result['status'], result['cleanup_error'] = 'UNVERIFIED', repr(error)
        if hold is not None:
            try:
                hold.stop()
            except Exception as error:
                result['status'], result['cleanup_error'] = 'UNVERIFIED', repr(error)
        result['remaining_identities'] = owned.stop(grace=30.)
        if result['remaining_identities']:
            result['status'] = 'UNVERIFIED'
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--network-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command, network, output = validate_request(args.command, args.network_config, args.output)
    import psutil
    output.mkdir(parents=True)
    lock = threading.Lock()
    def emit(event, **fields):
        # A full disk must never prevent emergency process/stop containment.
        try:
            with lock, (output/'events.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(event=event, monotonic=time.monotonic(),
                    wall_time=time.time(), **fields), allow_nan=False)+'\n')
        except (OSError, ValueError, TypeError) as error:
            try:
                print('Supervisor evidence unavailable: '+repr(error), file=sys.stderr, flush=True)
            except OSError:
                pass
    (output/'metadata.json').write_text(json.dumps(dict(command=command,
        network_config=str(network), supervisor_pid=os.getpid(),
        network_config_sha256=hashlib.sha256(network.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        supervisor_signal_SET=False, development_only=True,
        limitation='Supervisor death/SIGKILL or missed descendants cannot guarantee physical stop.'), indent=2)+'\n')
    stop = threading.Event()
    old_handlers = {sig: signal.signal(sig, lambda *_: stop.set())
                    for sig in (signal.SIGINT, signal.SIGTERM)}
    process = owned = None
    try:
        with (output/'owner.log').open('x') as logfile:
            process = subprocess.Popen(command, cwd=ROOT, stdout=logfile, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            owned = OwnedProcesses(process.pid, psutil.Process, emit)
            emit('owner_started', pid=process.pid, create_time=owned.owner_created)
            interrupted_at = None
            while process.poll() is None:
                owned.capture()
                if stop.is_set() and interrupted_at is None:
                    owner = owned.same_process(dict(pid=process.pid, create_time=owned.owner_created))
                    if owner is not None:
                        owner.send_signal(signal.SIGINT)
                    interrupted_at = time.monotonic()
                    emit('owner_interrupt_forwarded')
                if interrupted_at is not None and time.monotonic()-interrupted_at > 45.:
                    # Identity-checked owner, not any unrelated process group.
                    owner = owned.same_process(dict(pid=process.pid, create_time=owned.owner_created))
                    if owner is not None:
                        owner.send_signal(signal.SIGKILL)
                    process.wait(timeout=3.)
                    break
                stop.wait(.1) if not stop.is_set() else time.sleep(.1)
            emit('owner_exited', returncode=process.returncode)
            result = finish_trial(process.returncode, owned, network, output, emit)
            (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
            return 0 if result['status'] == 'NORMAL' else 2
    except BaseException as error:
        emit('supervisor_error', error=type(error).__name__+': '+str(error))
        if process is not None and owned is not None:
            try:
                stop_owner_for_recovery(process, owned, emit)
                result = finish_trial(process.returncode, owned, network, output, emit)
                result['supervisor_error'] = repr(error)
                try:
                    (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
                except OSError:
                    pass
            except BaseException as recovery_error:
                emit('supervisor_recovery_error', error=repr(recovery_error), stop_park_verified=False)
        return 2
    finally:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
