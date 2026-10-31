#!/usr/bin/env python3
"""Separately approved borrowed-host ROS and bounded capture executor.

The original preparation-only bundle is immutable, including can_run=false.
This executor adds a host-specific contract and independently checks native
smoke evidence before exposing an explicit, operator-started bulk command.
No install, shell profile, credential, GUI launch, or unconfined fallback.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import stat
import subprocess
import sys
import time

WORKSPACE = 'ajounice_base5_capture'
MANIFEST_SHA = 'f0f30909639e0f3123e6fc89ac64b01bb60a8b6445be4331b9e7efee2f1cb1f0'
RELEASE_SHA = '0a3dd5c96b4522bc9d0547bed845c97ebb02d228b30ff398e1d7ed688b726180'
DECK_SHA = '4f7fc3b31e23688b43913f2457d570cf014c19e04b2c2e391b463647809e02c5'
SMOKE_INDICES = (2461, 1679, 4857)
SMOKE_IDS = ('sunny13_sunny11_b5v2_02554', 'sunny13_sunny11_b5v2_01744',
             'sunny13_sunny11_b5v2_00486')
MR_SAVE = 'MoraiLauncher_Lin_Data/SaveFile'
SCENE_REL = 'Scenario/R_KR_PR_K-city_2025'
SCHEMA = 'base5_borrowed_live_contract_v1'


def require(value, message):
    if not value:
        raise ValueError(message)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024**2), b''):
            h.update(block)
    return h.hexdigest()


def no_symlinks(path):
    path = Path(path).absolute()
    require('..' not in path.parts, 'Normalized absolute path required')
    for component in (path, *path.parents):
        require(not component.is_symlink(), 'Symlink is not permitted: ' + str(component))
    return path


def owned(path, directory=True):
    path = no_symlinks(path)
    info = path.stat()
    require(info.st_uid == os.getuid(), 'Foreign-owned path: ' + str(path))
    require(stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode),
            'Unexpected path type: ' + str(path))
    require(not info.st_mode & 0o022, 'Group/world-writable input: ' + str(path))
    return path


def read(path):
    return json.loads(Path(path).read_text())


def write_new(path, value):
    path = no_symlinks(path)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def checked_folder(path):
    path = no_symlinks(path)
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    return owned(path)


def load(workspace, verify=True):
    workspace = owned(workspace)
    require(workspace.name == WORKSPACE and workspace.parent == Path.home().resolve(),
            'Only the approved workspace under the real operator HOME is supported')
    bundle = owned(workspace / 'capture_bundle')
    require(digest(bundle / 'portable_bundle.json') == MANIFEST_SHA, 'Bundle manifest changed')
    doc = read(bundle / 'portable_bundle.json')
    require(doc.get('can_run') is False, 'Historical preparation-only bundle was modified')
    graph = read(bundle / 'portable_graph.json')
    require(graph['release']['sha256'] == RELEASE_SHA and graph['manifest']['sha256'] == DECK_SHA,
            'Unexpected release or source deck')
    if verify:
        sys.path.insert(0, str(bundle / 'tools'))
        import runtime
        from portable_paths import verify_graph
        runtime.load_bundle(bundle)
        verify_graph(graph, bundle)
    mr = owned(workspace.parent / 'MoraiLauncher_Lin')
    for kind, name in [('Sensor', 'Sensor_competition_local.json'),
                       ('Network', 'ego_network_mr_base5_savesensordata_collision_local.json')]:
        original = bundle / 'morai/MoraiLauncher_Lin' / MR_SAVE / kind / name
        actual = mr / MR_SAVE / kind / '25.S4.MolitComp03' / name
        owned(actual, False)
        require(digest(original) == digest(actual), 'MORAI UI profile changed: ' + name)
    return dict(workspace=workspace, bundle=bundle, doc=doc, graph=graph, mr=mr,
                session=workspace / 'work/ros-session-v2', home=workspace / 'work/ros-home-v2')


def contract(ctx):
    return dict(schema=SCHEMA, workspace=str(ctx['workspace']), morai=str(ctx['mr']),
                bundle_sha256=MANIFEST_SHA, release_sha256=RELEASE_SHA,
                manifest_sha256=DECK_SHA, environment='sunny13',
                executor_sha256=digest(__file__), smoke_indices=list(SMOKE_INDICES),
                smoke_ids=list(SMOKE_IDS), maximum_smoke_captures=3,
                source_bundle_remains_preparation_only=True,
                bulk_started=False, automatic_completion_enabled=False)


def prepare(ctx):
    """Only add new owned runtime state; never edit the TXT or frozen bundle."""
    session, home = ctx['session'], ctx['home']
    folders = (session, home, session / 'log', session / 'tmp', home / '.cache',
               home / '.config', home / '.local/share', home / '.ros',
               ctx['mr'] / MR_SAVE / SCENE_REL, ctx['mr'] / MR_SAVE / SCENE_REL / 'EgoNetwork',
               ctx['mr'] / MR_SAVE / 'SensorData', ctx['workspace'] / 'collections')
    expected = contract(ctx)
    target = session / 'host_contract.json'
    if target.exists():
        require(read(target) == expected, 'Existing host contract differs; preserve it and inspect')
    alias = home / 'MoraiLauncher_Lin'
    if alias.is_symlink():
        require(alias.resolve(strict=True) == ctx['mr'], 'Existing MORAI alias differs')
    else:
        require(not alias.exists(), 'Existing HOME alias would be overwritten')
    # Runtime catalogs are frozen source inputs, not regenerated asset tables.
    catalog_files = []
    for entry in ctx['graph']['files'].values():
        if entry['source'].startswith('/home/luv/MoraiLauncher_Lin/'):
            relative = Path(entry['source']).relative_to('/home/luv/MoraiLauncher_Lin')
            source = ctx['bundle'] / 'payload' / Path(entry['source']).relative_to('/')
            destination = ctx['mr'] / relative
            no_symlinks(destination)
            if destination.exists():
                require(destination.is_file() and digest(destination) == entry['sha256'],
                        'Preserving an existing different runtime catalog')
            catalog_files.append((source, destination, entry['sha256']))
    for folder in folders:
        checked_folder(folder)
    if not alias.is_symlink():
        alias.symlink_to(ctx['mr'], target_is_directory=True)
    for source, destination, expected_sha in catalog_files:
        if not destination.exists():
            subprocess.run(['/usr/bin/cp', '--no-clobber', '--no-target-directory', '--',
                            str(source), str(destination)], check=True)
        require(digest(destination) == expected_sha, 'Runtime catalog verification failed')
    if not target.exists():
        write_new(target, expected)
    return expected


def ready(ctx):
    owned(ctx['session'])
    owned(ctx['home'])
    require(read(ctx['session'] / 'host_contract.json') == contract(ctx), 'Host contract differs')
    require((ctx['home'] / 'MoraiLauncher_Lin').is_symlink()
            and (ctx['home'] / 'MoraiLauncher_Lin').resolve(strict=True) == ctx['mr'],
            'Approved runtime MORAI alias is missing')


def environment(ctx):
    bundle = Path('/run/base5-bundle')
    overlay = bundle / 'overlay'
    repo = Path('/home/luv/AjouNice2026')
    ros = Path('/opt/ros/noetic')
    home, session = ctx['home'], ctx['session']
    return dict(PATH=str(ros / 'bin') + ':/usr/local/bin:/usr/bin:/bin',
        LANG='C.UTF-8', LC_ALL='C.UTF-8', HOME=str(home),
        ROS_HOME=str(home / '.ros'), ROS_LOG_DIR=str(session / 'log'),
        ROS_MASTER_URI='http://127.0.0.1:11311', ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
        ROS_DISTRO='noetic', ROS_VERSION='1', ROS_PYTHON_VERSION='3',
        ROS_ETC_DIR=str(ros / 'etc/ros'),
        ROS_PACKAGE_PATH=str(overlay / 'share') + ':' + str(ros / 'share'),
        CMAKE_PREFIX_PATH=str(overlay) + ':' + str(ros),
        LD_LIBRARY_PATH=str(overlay / 'lib') + ':' + str(ros / 'lib'),
        PYTHONPATH=':'.join(map(str, [bundle / 'python', overlay / 'lib/python3/dist-packages',
                    repo / 'dataset/morai_od/scripts', repo, ros / 'lib/python3/dist-packages'])),
        PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        TMPDIR=str(session / 'tmp'), TMP=str(session / 'tmp'), TEMP=str(session / 'tmp'),
        XDG_CACHE_HOME=str(home / '.cache'), XDG_CONFIG_HOME=str(home / '.config'),
        XDG_DATA_HOME=str(home / '.local/share'), OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')


def namespace(ctx, command):
    """Same host loopback; no host HOME, agent, devices or unconfined fallback."""
    bundle, workspace, mr = ctx['bundle'], ctx['workspace'], ctx['mr']
    args = [str(bundle / 'bin/bwrap'), '--unshare-user', '--unshare-pid',
            '--die-with-parent', '--new-session', '--ro-bind', '/', '/',
            '--proc', '/proc', '--dev', '/dev', '--tmpfs', '/tmp',
            '--tmpfs', '/home', '--tmpfs', '/root', '--tmpfs', '/run',
            '--dir', str(workspace.parent), '--dir', '/home/luv',
            '--bind', str(workspace), str(workspace), '--bind', str(mr), str(mr),
            '--ro-bind', str(bundle), str(bundle),
            '--ro-bind', str(bundle), '/run/base5-bundle']
    for binding in ctx['graph']['bindings']:
        args += ['--ro-bind', str(bundle / binding['bundle_path']), binding['logical_root']]
    args += ['--chdir', '/home/luv/AjouNice2026', '--', *command]
    return args


def port_open(port):
    with socket.socket() as sock:
        sock.settimeout(.4)
        return sock.connect_ex(('127.0.0.1', port)) == 0


def service_owned(ctx, role):
    """A foreign listening port is not authority to use that ROS environment."""
    path = ctx['session'] / (role + '.lock')
    if not path.is_file() or path.is_symlink():
        return False
    with path.open('r') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            pass
        else:
            fcntl.flock(lock, fcntl.LOCK_UN)
            return False
    records = sorted(ctx['session'].glob(role + '_*.json'))
    if not records:
        return False
    record = read(records[-1])
    proc = Path('/proc') / str(record['supervisor_pid'])
    try:
        argv = (proc / 'cmdline').read_bytes().split(b'\x00')
        start = (proc / 'stat').read_text().rsplit(')', 1)[1].split()[19]
        return (proc.stat().st_uid == os.getuid() and record.get('action') == role
                and record.get('executor_sha256') == digest(__file__)
                and record.get('pid_start_ticks') == start
                and os.fsencode(str(Path(__file__).resolve())) in argv
                and role.encode() in argv and b'--run' in argv)
    except OSError:
        return False


def request_pause(ctx):
    root = checked_folder(ctx['workspace'] / 'collections/sunny13_5k')
    marker = no_symlinks(root / 'PAUSE')
    if marker.exists():
        owned(marker, False)
    else:
        write_new(marker, dict(reason='operator_graceful_pause', requested_at=time.time(),
                               semantics='finish current bounded chunk and native Park cleanup'))
    return marker


def application_argv(argv, command):
    actual = [os.fsdecode(value) for value in argv if value]
    if actual == command:
        return True
    # roscore is a Python shebang script; bwrap's own argv must never match.
    return (command[0].startswith('/opt/ros/') and len(actual) == len(command) + 1
            and Path(actual[0]).name.startswith('python3') and actual[1:] == command)


def signal_inner_app(child, command, signum):
    """Signal the actual application, never the bwrap namespace supervisor.

    killpg(bwrap_pid, SIGINT) can tear down its namespace before native Park.
    Use the exact argv in the owned descendant tree and a pidfd (no PID reuse).
    """
    queue, seen, candidates = [child.pid], set(), []
    while queue:
        pid = queue.pop()
        if pid in seen:
            continue
        seen.add(pid)
        proc = Path('/proc') / str(pid)
        try:
            require(proc.stat().st_uid == os.getuid(), 'Foreign process in runtime tree')
            queue.extend(map(int, (proc / ('task/' + str(pid) + '/children')).read_text().split()))
            argv = (proc / 'cmdline').read_bytes().split(b'\x00')
            if application_argv(argv, command):
                start = (proc / 'stat').read_text().rsplit(')', 1)[1].split()[19]
                candidates.append((pid, start))
        except (FileNotFoundError, ProcessLookupError):
            continue
    require(len(candidates) <= 1, 'Ambiguous runtime application; do not send a broad signal')
    if not candidates:
        return False
    require(hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'),
            'Linux pidfd is required for safe interruption')
    pid, expected_start = candidates[0]
    try:
        fd = os.pidfd_open(pid)
    except ProcessLookupError:
        return False
    try:
        # Recheck identity after opening the stable process reference.
        proc = Path('/proc') / str(pid)
        argv = (proc / 'cmdline').read_bytes().split(b'\x00')
        start = (proc / 'stat').read_text().rsplit(')', 1)[1].split()[19]
        require(application_argv(argv, command) and proc.stat().st_uid == os.getuid()
                and start == expected_start, 'Application identity changed')
        signal.pidfd_send_signal(fd, signum)
    except ProcessLookupError:
        return False
    finally:
        os.close(fd)
    return True


INSPECT_CODE = r'''
import json,time,threading,rospy
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CollisionData
rospy.init_node('base5_portable_connection_check',anonymous=True,disable_signals=True)
rows={}; counts={}; times={}; lock=threading.Lock()
def callback(msg,topic):
    with lock:
        now=time.monotonic();counts[topic]=counts.get(topic,0)+1;times[topic]=now
        if topic=='/Ego_topic':
            rows[topic]=dict(x=msg.position.x,y=msg.position.y,z=msg.position.z,
                heading=msg.heading,vx=msg.velocity.x,vy=msg.velocity.y,vz=msg.velocity.z,brake=msg.brake)
        elif topic=='/Object_topic':
            rows[topic]=dict(npc=len(msg.npc_list),pedestrian=len(msg.pedestrian_list),obstacle=len(msg.obstacle_list))
        else:rows[topic]=dict(observed=True)
subscribers=[rospy.Subscriber(t,c,callback,callback_args=t,queue_size=10) for t,c in
    [('/Ego_topic',EgoVehicleStatus),('/Object_topic',ObjectStatusList),('/CollisionData',CollisionData)]]
started=time.monotonic()
while time.monotonic()-started<5 and not rospy.is_shutdown():time.sleep(.05)
now=time.monotonic()
out=dict(event='portable_connection_observed',elapsed_sec=now-started,topics=rows,
    counts=counts,age_sec={k:now-v for k,v in times.items()},control_commands_sent=0,captures_requested=0)
print(json.dumps(out),flush=True)
rospy.signal_shutdown('read-only connection check finished')
if any(counts.get(k,0)<2 or now-times.get(k,0)>1 for k in ('/Ego_topic','/Object_topic','/CollisionData')):raise SystemExit(2)
'''


def action_command(ctx, action):
    scripts = Path('/home/luv/AjouNice2026/dataset/morai_od/scripts')
    if action == 'ros':
        return ['/opt/ros/noetic/bin/roscore', '-p', '11311']
    if action == 'bridge':
        return ['/usr/bin/python3', '-B', str(scripts / 'run_base5_rosbridge.py'),
                '_address:=127.0.0.1', '_port:=9090']
    if action == 'inspect':
        return ['/usr/bin/python3', '-B', '-c', INSPECT_CODE]
    if action == 'smoke':
        return ['/usr/bin/python3', '-B', str(scripts / 'run_base5_capture_optimized.py'),
                '--setup-profile', 'static-ready', '--manifest', ctx['graph']['manifest']['source'],
                '--indices', ','.join(map(str, SMOKE_INDICES)), '--limit', '3',
                '--shared-ros-batch', '--output', str(ctx['workspace'] / 'collections/smoke_v1')]
    if action == 'capture':
        return ['/usr/bin/python3', '-B', str(scripts / 'launch_base5_obstacle_support.py'),
                '--release', ctx['graph']['release']['source'], '--environment', 'sunny13',
                '--output', str(ctx['workspace'] / 'collections/sunny13_5k'),
                '--run', '--confirmed-environment', 'sunny13']
    raise ValueError('Unknown live action')


def supervise(ctx, action, command):
    lock_path = no_symlinks(ctx['session'] / (action + '.lock'))
    if lock_path.exists():
        owned(lock_path, False)
    fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        args, env = namespace(ctx, command), environment(ctx)
        if action == 'smoke':
            budget = ctx['session'] / 'smoke_attempt.json'
            require(not budget.exists(), 'Three-capture smoke budget already reserved; do not retry automatically')
            write_new(budget, dict(indices=list(SMOKE_INDICES), maximum_captures=3,
                                  reserved_at=time.time(), state='reserved_before_process_start'))
        child = None
        previous = {}
        def forward(signum, frame):
            if child is not None and child.poll() is None:
                if action == 'capture':
                    marker = request_pause(ctx)
                    print(json.dumps(dict(event='graceful_pause_requested', marker=str(marker),
                        note='Wait for current chunk and Park; keep ROS/bridge running')), flush=True)
                else:
                    signal_inner_app(child, command, signal.SIGINT if signum == signal.SIGHUP else signum)
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[signum] = signal.signal(signum, forward)
        try:
            child = subprocess.Popen(args, env=env, start_new_session=True)
            start = Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()[19]
            receipt = ctx['session'] / (action + '_' + str(time.time_ns()) + '.json')
            write_new(receipt, dict(event='approved_process_started', action=action,
                      supervisor_pid=os.getpid(), pid_start_ticks=start,
                      child_pid=child.pid, time=time.time(), command=command,
                      executor_sha256=digest(__file__), scope='approved two folders only'))
            return child.wait()
        finally:
            if child is not None and child.poll() is None:
                if action == 'capture':
                    request_pause(ctx)
                else:
                    signal_inner_app(child, command, signal.SIGINT)
                # Do not SIGKILL a collector before its native Park cleanup.
                child.wait()
            for signum, handler in previous.items():
                signal.signal(signum, handler)


def qualify(ctx):
    """Read actual native/exit evidence, never fill in missing proof."""
    root = ctx['workspace'] / 'collections/smoke_v1'
    status = read(root / 'status.json')
    require(status.get('stage') == 'complete' and status.get('accepted_candidate') == 3
            and status.get('review') == 0 and status.get('failed') == 0,
            'Exactly three accepted native smoke frames are required')
    rows = [json.loads(line) for line in (root / 'ledger.jsonl').read_text().splitlines()]
    require(len(rows) == 3 and [r['id'] for r in rows] == list(SMOKE_IDS)
            and all(r.get('status') == 'accepted_candidate' for r in rows), 'Smoke ledger differs')
    # The predicate checks the recorded Park reply. Independently reconstruct
    # session/native payload QA below; do not call that predicate a rest proof.
    code = r'''
import json
from pathlib import Path
from base5_capture_safety import verified_shutdown,prepare_session,sha256
from base5_capture_audit import audit_capture
root=Path(ROOT)
scripts=Path('/home/luv/AjouNice2026/dataset/morai_od/scripts')
assert (root/'session_contract.json').is_file() and (root/'ledger.jsonl').is_file()
assert verified_shutdown(root/'shutdown_safety.json')
options=dict(shared_ros_batch=True,selected_scene_ids=IDS,optimized_adapter=dict(
    profile='static-ready',adapter_sha256=sha256(scripts/'run_base5_capture_optimized.py'),
    source_sha256=sha256(scripts/'run_base5_capture.py'),
    policy='static conditional setup delay only; native gates'))
repo=scripts.parents[2]
rows,completed=prepare_session(root,Path(MANIFEST),
    repo/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
    repo/'configs/sensors/Sensor_competition_local.json',options)
assert len(rows)==len(completed)==3 and list(completed)==IDS
for row in rows:
    capture=Path(row['capture_root'])
    assert capture.resolve().is_relative_to(root.resolve())
    meta=json.loads((capture/'metadata.json').read_text())
    audit=audit_capture(capture,meta)
    assert audit['accepted_candidate'],audit['review_reasons']
    assert audit.get('capture_safety',{}).get('collision',{}).get('no_collision_verified')
    assert audit.get('capture_safety',{}).get('support',{}).get('stationary_support_observed')
    assert row.get('between_scene_park',{}).get('park_verified') is True
    for camera in range(1,5):
        assert len(list((capture/('CAMERA_'+str(camera))).glob('*_Instance.png')))==1
print(json.dumps(dict(event='native_smoke_reconstruction_pass',frames=3)))
'''
    code = ('ROOT=' + repr(str(root)) + '\nMANIFEST=' + repr(ctx['graph']['manifest']['source'])
            + '\nIDS=' + repr(list(SMOKE_IDS)) + '\n' + code)
    subprocess.run(namespace(ctx, ['/usr/bin/python3', '-B', '-c', code]),
                   env=environment(ctx), check=True, timeout=60)
    files = {}
    for path in sorted(root.rglob('*')):
        require(not path.is_symlink(), 'Symlink in smoke evidence')
        if path.is_file():
            files[str(path.relative_to(root))] = digest(path)
    proof = dict(schema='base5_borrowed_host_native_qualification_v1', host_contract=contract(ctx),
                 ledger_sha256=digest(root / 'ledger.jsonl'), status_sha256=digest(root / 'status.json'),
                 shutdown_sha256=digest(root / 'shutdown_safety.json'), source_ids=list(SMOKE_IDS),
                 native_frames=3, native_files=files, bulk_started=False, qualified_at=time.time())
    destination = ctx['session'] / 'qualification.json'
    require(not destination.exists(), 'Existing host qualification is preserved')
    write_new(destination, proof)
    return proof


def verify_qualification(ctx):
    proof = read(ctx['session'] / 'qualification.json')
    require(proof.get('host_contract') == contract(ctx), 'Qualified executor or host contract changed')
    root = ctx['workspace'] / 'collections/smoke_v1'
    for field, filename in [('ledger_sha256', 'ledger.jsonl'), ('status_sha256', 'status.json'),
                            ('shutdown_sha256', 'shutdown_safety.json')]:
        require(proof[field] == digest(root / filename), 'Native smoke evidence changed')
    require(proof.get('native_frames') == 3, 'Missing native frame qualification')
    for relative, expected in proof['native_files'].items():
        require(not Path(relative).is_absolute() and '..' not in Path(relative).parts,
                'Invalid native evidence path')
        require(digest(root / relative) == expected, 'Native smoke payload changed')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path.home() / WORKSPACE)
    parser.add_argument('action', choices=['prepare', 'check', 'ros', 'bridge', 'inspect', 'smoke', 'qualify', 'capture'])
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--confirmed-environment', choices=['sunny13'])
    args = parser.parse_args(argv)
    ctx = load(args.workspace)
    if args.action == 'check':
        print(json.dumps(dict(event='borrowed_live_inputs_checked', contract=contract(ctx),
                             port11311=port_open(11311), port9090=port_open(9090),
                             files_written=False, capture_started=False), ensure_ascii=False, indent=2))
        return 0
    if not args.run:
        print(json.dumps(dict(event='borrowed_live_plan_only', action=args.action,
              command=action_command(ctx, args.action) if args.action not in ('prepare', 'qualify') else None,
              files_written=False, capture_started=False), ensure_ascii=False, indent=2))
        return 0
    if args.action == 'prepare':
        print(json.dumps(prepare(ctx), ensure_ascii=False, indent=2))
        return 0
    ready(ctx)
    if args.action == 'qualify':
        print(json.dumps(qualify(ctx), ensure_ascii=False, indent=2))
        return 0
    if args.action == 'ros':
        require(not port_open(11311), 'Existing ROS master: do not reuse or stop foreign processes')
    else:
        require(port_open(11311), 'Start the approved ROS terminal first')
        require(service_owned(ctx, 'ros'), 'Listening ROS master is not owned by this approved runtime')
    if args.action == 'bridge':
        require(not port_open(9090), 'Existing bridge: do not reuse or stop foreign processes')
    elif args.action in ('inspect', 'smoke', 'capture'):
        require(port_open(9090), 'Start the approved bridge terminal first')
        require(service_owned(ctx, 'bridge'), 'Listening bridge is not owned by this approved runtime')
    if args.action in ('smoke', 'capture'):
        require(args.confirmed_environment == 'sunny13', 'Confirm actual MR UI Sunny13 before capture')
        if args.action == 'capture':
            verify_qualification(ctx)
    return supervise(ctx, args.action, action_command(ctx, args.action))


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(json.dumps(dict(event='borrowed_live_blocked', error=str(error),
                              fallback_allowed=False), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
