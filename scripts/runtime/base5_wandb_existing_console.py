#!/usr/bin/env python3
"""Explicit files-only console restoration after the SDK writer is disabled.

Never upserts a run, logs metrics, sends heartbeats, or completes a W&B run.
The authority, process-start identity, and WANDB_CONSOLE=off guard are checked
before every output.log POST. Training and frozen SDK code remain untouched.
"""
import argparse
import fcntl
import hashlib
import json
import netrc
import os
from pathlib import Path
import sys
import time
import urllib.error

from base5_wandb_console_mirror import (
    Mirror, MirrorError, RetryableError, WandbAPI, check_secrets, digest,
    encoded, utc_now)


def authority(path):
    p = Path(path)
    raw = p.read_bytes()
    a = json.loads(raw)
    allowed = {'authorized', 'entity', 'project', 'run', 'contract_sha256',
               'training_status', 'guard_entry_receipt', 'prelude', 'newlog',
               'state_dir', 'poll_interval_seconds', 'config_binding'}
    if set(a) - allowed or a.get('authorized') is not True:
        raise MirrorError('explicit existing-console authority required')
    import re
    for key in ('entity', 'project', 'run'):
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', a.get(key, '')):
            raise MirrorError('invalid existing run identity')
    if not re.fullmatch('[a-f0-9]{64}', a.get('contract_sha256', '')):
        raise MirrorError('contract SHA256 required')
    for key in ('training_status', 'guard_entry_receipt', 'newlog', 'state_dir'):
        if not Path(a.get(key, '')).is_absolute():
            raise MirrorError('absolute scoped path required')
    prelude = a.get('prelude', {})
    if set(prelude) != {'path', 'sha256'} or not Path(prelude['path']).is_absolute() \
            or not re.fullmatch('[a-f0-9]{64}', prelude['sha256']):
        raise MirrorError('immutable prelude path and SHA256 required')
    interval = a.get('poll_interval_seconds', 20)
    if not 1 <= interval <= 300:
        raise MirrorError('poll interval out of range')
    a['poll_interval_seconds'] = interval
    binding = a.get('config_binding', {})
    if set(binding) != {'storage_authority_sha256'} or not re.fullmatch('[a-f0-9]{64}', binding['storage_authority_sha256']):
        raise MirrorError('explicit storage authority SHA256 binding required')
    a['_authority_path'] = str(p.resolve(strict=True))
    return a, digest(raw)


class WriterGuard:
    def __init__(self, a, authority_sha256):
        self.a, self.sha = a, authority_sha256

    def __call__(self):
        if self.a.get('_authority_path') and digest(Path(self.a['_authority_path']).read_bytes()) != self.sha:
            raise MirrorError('console authority changed after loading')
        receipt = json.loads(Path(self.a['guard_entry_receipt']).read_text())
        if (receipt.get('authority_sha256') != self.a['config_binding']['storage_authority_sha256']
                or receipt.get('contract_sha256') != self.a['contract_sha256']
                or receipt.get('wandb_console') != 'off'
                or receipt.get('event') != 'ENTRY_VALIDATED'
                or receipt.get('complete') is not True):
            raise MirrorError('console writer guard receipt binding differs')
        pid, start = receipt.get('pid'), receipt.get('pid_start_ticks')
        if type(pid) is not int or pid <= 1 or not isinstance(start, (int, str)):
            raise MirrorError('actual training PID/start identity required')
        proc = Path('/proc') / str(pid)
        try:
            # comm may contain spaces or parentheses; fields follow its last ).
            actual_start = proc.joinpath('stat').read_text().rsplit(')', 1)[1].split()[19]
            if str(start) == actual_start:
                console = None
                for item in proc.joinpath('environ').read_bytes().split(b'\0'):
                    key, _, value = item.partition(b'=')
                    if key == b'WANDB_CONSOLE':
                        console = value
                if console != b'off':
                    raise MirrorError('live SDK console writer is not disabled')
                return
        except FileNotFoundError:
            pass
        status = json.loads(Path(self.a['training_status']).read_text()).get('state')
        if status not in ('finished', 'failed', 'completed', 'succeeded'):
            raise MirrorError('training PID disappeared without terminal status; hold writes')


class ExistingAPI(WandbAPI):
    def __init__(self, a, key, guard):
        super().__init__(dict(entity=a['entity'], project=a['project'], mirror_run=a['run']), key)
        self.a, self.guard = a, guard

    def check_run(self):
        query = ('query($entity:String!,$project:String!,$run:String!){'
                 'project(name:$project,entityName:$entity){run(name:$run){name config state}}}')
        data = self.graphql(query, {k: self.a[k] for k in ('entity', 'project', 'run')})
        run = (data.get('project') or {}).get('run')
        if not run or run.get('name') != self.a['run']:
            raise MirrorError('existing run identity differs')
        config = run.get('config') or '{}'
        if self.a['contract_sha256'] not in (config if isinstance(config, str) else json.dumps(config)):
            raise MirrorError('existing run configuration does not contain the bound contract')

    def append(self, offset, lines):
        self.guard()
        for line in lines:
            check_secrets(line, self.key)
        # Deliberately no complete/exitcode/history/events/summary fields.
        self.request('/files/' + self.run_path + '/file_stream',
                     {'files': {'output.log': {'offset': offset, 'content': lines}}})

    def ensure_run(self):
        raise MirrorError('run metadata changes are forbidden')

    def heartbeat(self):
        return None

    def complete(self):
        # Only the training SDK owns the original run lifecycle.
        return None


class ExistingMirror(Mirror):
    def __init__(self, a, authority_sha, api, key='', clock=utc_now):
        c = dict(a, origin_run=a['run'], mirror_run=a['run'], authority_sha256=authority_sha,
                 sources=[dict(path=a['prelude']['path'], immutable=True, sha256=a['prelude']['sha256']),
                          dict(path=a['newlog'], immutable=False)])
        super().__init__(c, api, key, clock)

    def build(self, data, timestamp, max_lines=1000):
        # A file-backed worker log includes its startup config. Drain that
        # bounded backlog without hundreds of redundant full-console reads.
        return super().build(data, timestamp, max_lines=max_lines)

    @staticmethod
    def compatible_restore(server, original, wanted):
        # Offset zero can leave an old tail until all replacement lines land.
        # Accept only the exact old snapshot or an exact replacement prefix
        # followed by that snapshot's unchanged suffix; never an unknown writer.
        n = 0
        while n < min(len(server), len(wanted)) and server[n] == wanted[n]:
            n += 1
        return (server == wanted or
                (n == len(server) and len(server) >= len(original)) or
                (len(server) == len(original) and server[n:] == original[n:]))

    def restore(self, data):
        if self.s.get('prelude_restored'):
            return False
        self.api.guard()
        raw = data[0]
        if not raw.endswith(b'\n'):
            raise MirrorError('immutable prelude must end in a complete line')
        text = raw.decode('utf-8')
        check_secrets(text, self.key)
        wanted = text.splitlines(keepends=True)
        server = self.api.output()
        snapshot = self.dir / 'original_console_snapshot.json'
        restore = self.s.get('restore_transaction')
        if restore is None:
            check_secrets(''.join(server), self.key)
            if len(wanted) <= len(server):
                raise MirrorError('prelude must exceed current console length to remove every old tail')
            with open(snapshot, 'x', encoding='utf-8') as f:
                os.chmod(snapshot, 0o600)
                json.dump(server, f, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            restore = dict(original_sha256=digest(encoded(server)), original_count=len(server),
                           timestamp=self.clock(), prelude_sha256=digest(raw), post_intent=False)
            self.s['restore_transaction'] = restore
            self.save()
        original = json.loads(snapshot.read_text())
        if digest(encoded(original)) != restore['original_sha256'] or len(original) != restore['original_count']:
            raise MirrorError('saved original console snapshot changed')
        if digest(raw) != restore['prelude_sha256']:
            raise MirrorError('pending immutable prelude changed')
        if server != wanted:
            if not restore['post_intent'] and server != original:
                raise MirrorError('server console changed before authorized restoration')
            if not self.compatible_restore(server, original, wanted):
                raise MirrorError('unexpected server console during restoration')
            restore['post_intent'] = True
            self.save()
            self.api.append(0, [restore['timestamp'] + ' ' + line for line in wanted])
            server = self.api.output()
            if server != wanted:
                if self.compatible_restore(server, original, wanted):
                    raise RetryableError('prelude readback not fully visible yet')
                raise MirrorError('restored prelude readback differs')
        self.s.update(prelude_restored=True, cursors=[len(raw), 0], boundary=True,
                      count=len(wanted), sha256=digest(encoded(wanted)), pending=None)
        self.s['sources'][0]['prefix_sha256'] = digest(raw)
        self.s['sources'][1]['prefix_sha256'] = digest(b'')
        self.save()
        return True

    def poll(self):
        self.api.guard()
        data = self.source_data()
        for i, src in enumerate(data):
            check_secrets(src[self.s['cursors'][i]:].rsplit(b'\n', 1)[0].decode('utf-8'), self.key)
        changed = self.restore(data)
        return super().poll() or changed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--authority', required=True)
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args(argv)
    try:
        a, sha = authority(args.authority)
        key = os.environ.get('WANDB_API_KEY')
        if not key:
            auth = netrc.netrc().authenticators('api.wandb.ai')
            key = auth[2] if auth else None
        if not key:
            raise MirrorError('W&B authentication unavailable')
        directory = Path(a['state_dir'])
        directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        with open(directory / 'existing_console.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            guard = WriterGuard(a, sha)
            guard()
            api = ExistingAPI(a, key, guard)
            api.check_run()
            mirror = ExistingMirror(a, sha, api, key)
            while True:
                try:
                    changed = mirror.poll()
                except (urllib.error.URLError, TimeoutError, ConnectionError, RetryableError):
                    print(json.dumps({'event': 'existing_console_transport_retry'}), flush=True)
                    if args.once:
                        return 2
                    changed = False
                print(json.dumps({'event': 'existing_console_status', 'lines': mirror.s['count'],
                                  'local_complete': mirror.s['complete']}), flush=True)
                if args.once or mirror.s['complete']:
                    return 0
                if not changed:
                    time.sleep(a['poll_interval_seconds'])
    except Exception as exc:
        print(json.dumps({'event': 'existing_console_failed', 'error_type': type(exc).__name__}), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
