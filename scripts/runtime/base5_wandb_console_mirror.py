#!/usr/bin/env python3
"""Mirror console text to a distinct, explicitly authorized W&B run.

This stdlib sidecar never imports the training process or writes its run. It
does not log metrics, change checkpoints, or modify either input file. Its
durable transaction cursor permits retry after an ambiguous HTTP response.
"""
import argparse
import base64
import datetime
import fcntl
import hashlib
import json
import netrc
import os
from pathlib import Path
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


class MirrorError(RuntimeError):
    pass


class RetryableError(MirrorError):
    pass


def digest(data):
    return hashlib.sha256(data).hexdigest()


def encoded(lines):
    return ''.join(lines).encode('utf-8')


def canonical(lines):
    # Only outbound lines have our additional timestamp. GraphQL returns the
    # content without it; never apply this conversion twice to server content.
    result = []
    for line in lines:
        line = re.sub(r'^\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d(?:\.\d+)?(?:Z|\+00:00) ', '', line)
        result.append(line.rstrip('\n') + '\n')
    return result


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat().replace('+00:00', 'Z')


def check_secrets(text, api_key=''):
    patterns = (
        r'-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----',
        r'(?i)authorization\s*[:=]\s*[\"\x27]?(?:bearer|basic)\s+\S+',
        r'(?i)(?:WANDB_API_KEY|access_token|refresh_token|client_secret|api_key)'
        r'[\"\x27]?\s*[:=]\s*[\"\x27]?([A-Za-z0-9_./+\-]{12,})',
        r'\b(?:gh[pousr]_[A-Za-z0-9]{20,}|ya29\.[A-Za-z0-9_.-]{20,}|AKIA[A-Z0-9]{16})\b',
    )
    if (api_key and api_key in text) or any(re.search(p, text) for p in patterns):
        raise MirrorError('credential-like text detected; no console upload performed')


def load_config(path):
    c = json.loads(Path(path).read_text())
    allowed = {'allow_run_create', 'entity', 'project', 'origin_run', 'mirror_run',
               'display_name', 'sources', 'state_dir', 'training_status',
               'poll_interval_seconds', 'group'}
    if set(c) - allowed or c.get('allow_run_create') is not True:
        raise MirrorError('unknown config key or missing explicit run-create authority')
    for k in ('entity', 'project', 'origin_run', 'mirror_run'):
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', c.get(k, '')):
            raise MirrorError('invalid run identity')
    if c['origin_run'] == c['mirror_run']:
        raise MirrorError('original run is never a writable target')
    sources = c.get('sources', [])
    if len(sources) != 2 or sources[0].get('immutable') is not True or sources[1].get('immutable') is not False:
        raise MirrorError('exactly one immutable source then one growing source required')
    for s in sources:
        if set(s) - {'path', 'immutable', 'sha256'} or not Path(s.get('path', '')).is_absolute():
            raise MirrorError('invalid source specification')
    if not re.fullmatch('[a-f0-9]{64}', sources[0].get('sha256', '')):
        raise MirrorError('immutable source SHA256 must be pinned')
    if not Path(c.get('state_dir', '')).is_absolute():
        raise MirrorError('state directory must be absolute')
    if c.get('training_status') and not Path(c['training_status']).is_absolute():
        raise MirrorError('status path must be absolute')
    c.setdefault('poll_interval_seconds', 20)
    if not 1 <= c['poll_interval_seconds'] <= 300:
        raise MirrorError('poll interval out of bounds')
    c.setdefault('display_name', c['mirror_run'])
    return c


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise MirrorError('redirect refused')


class WandbAPI:
    """Only the fixed W&B origin is contacted; ambient proxies are disabled."""
    def __init__(self, config, api_key):
        self.c = config
        self.key = api_key
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
        self.auth = 'Basic ' + base64.b64encode(('api:' + api_key).encode()).decode()
        self.run_path = '/'.join(urllib.parse.quote(config[k], safe='') for k in ('entity', 'project', 'mirror_run'))

    def request(self, path, payload=None, missing_ok=False):
        if not path.startswith(('/graphql', '/files/')) or '?' in path:
            raise MirrorError('unexpected API path')
        data = None if payload is None else json.dumps(payload).encode()
        req = urllib.request.Request('https://api.wandb.ai' + path, data=data,
                                     headers={'Authorization': self.auth, 'Content-Type': 'application/json'})
        try:
            with self.opener.open(req, timeout=30) as response:
                raw = response.read(32 * 1024 * 1024 + 1)
                if len(raw) > 32 * 1024 * 1024:
                    raise MirrorError('console response exceeds size limit')
                return raw
        except urllib.error.HTTPError as exc:
            if missing_ok and exc.code == 404:
                return b''
            if exc.code in (408, 429, 500, 502, 503, 504):
                raise RetryableError('transient W&B HTTP response') from None
            raise MirrorError('W&B HTTP request failed (status %s)' % exc.code) from None

    def graphql(self, query, variables, allow_missing_run=False):
        body = json.loads(self.request('/graphql', {'query': query, 'variables': variables}))
        if allow_missing_run and body.get('errors'):
            errors = body['errors']
            if all(re.search(r'could not find run|run .* not found', e.get('message', ''), re.I) for e in errors):
                return {'project': {'run': None}}
        if body.get('errors') or 'data' not in body:
            raise MirrorError('W&B GraphQL request failed')
        return body['data']

    def ensure_run(self):
        c = self.c
        origin = 'https://wandb.ai/%s/%s/runs/%s' % (c['entity'], c['project'], c['origin_run'])
        query = ('query($entity:String!,$project:String!,$run:String!){'
                 'project(name:$project,entityName:$entity){run(name:$run){name config}}}')
        data = self.graphql(query, {'entity': c['entity'], 'project': c['project'], 'run': c['mirror_run']},
                            allow_missing_run=True)
        existing = (data.get('project') or {}).get('run')
        expected = {'origin_run_url': {'value': origin}}
        if existing:
            cfg = existing.get('config') or '{}'
            if isinstance(cfg, str):
                cfg = json.loads(cfg)
            if cfg != expected:
                raise MirrorError('existing mirror run is not owned by this console recovery')
            return
        inp = {'name': c['mirror_run'], 'modelName': c['project'], 'entityName': c['entity'],
               'displayName': c['display_name'], 'state': 'running', 'jobType': 'console_recovery',
               'notes': origin + '\nConsole-only recovery; no AP or training metrics.',
               'config': json.dumps(expected)}
        if c.get('group'):
            inp['groupName'] = c['group']
        self.graphql('mutation($input:UpsertBucketInput!){upsertBucket(input:$input){bucket{name}}}', {'input': inp})

    def output(self):
        query = ('query($entity:String!,$project:String!,$run:String!,$after:String){'
                 'project(name:$project,entityName:$entity){run(name:$run){logLineCount '
                 'logLines(first:500,after:$after,useImprovedPagination:true){'
                 'edges{cursor node{number timestamp line}} pageInfo{hasNextPage endCursor}}}}}')
        rows, after, seen = [], None, set()
        while True:
            data = self.graphql(query, {'entity': self.c['entity'], 'project': self.c['project'],
                                       'run': self.c['mirror_run'], 'after': after})
            run = (data.get('project') or {}).get('run')
            if not run:
                raise MirrorError('mirror run missing during console readback')
            # logLineCount is eventually-consistent run metadata: a real probe
            # returned 0 while all 200 newly streamed rows were readable. The
            # contiguous row numbers and improved-pagination tail are authority.
            connection = run['logLines']
            for edge in connection['edges']:
                if edge['cursor'] in seen:
                    raise MirrorError('repeated server console pagination cursor')
                if edge['node']['number'] != len(rows):
                    raise MirrorError('non-contiguous server console row numbers')
                seen.add(edge['cursor'])
                rows.append(edge['node']['line'].rstrip('\n') + '\n')
            page = connection['pageInfo']
            if not page['hasNextPage']:
                break
            if not page['endCursor'] or page['endCursor'] == after or len(rows) > 100000:
                raise MirrorError('invalid server console pagination')
            after = page['endCursor']
        # node.line is already content-only: do not strip a second ISO timestamp
        # belonging to the original logged message.
        return rows

    def append(self, offset, lines):
        self.request('/files/' + self.run_path + '/file_stream',
                     {'files': {'output.log': {'offset': offset, 'content': lines}}, 'complete': False})

    def heartbeat(self):
        self.request('/files/' + self.run_path + '/file_stream', {'files': {}, 'complete': False})

    def complete(self):
        self.request('/files/' + self.run_path + '/file_stream', {'files': {}, 'complete': True, 'exitcode': 0})


class Mirror:
    def __init__(self, config, api, api_key='', clock=utc_now):
        self.c, self.api, self.key, self.clock = config, api, api_key, clock
        self.dir = Path(config['state_dir'])
        self.dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.path = self.dir / 'console_mirror_state.json'
        binding = digest(json.dumps(config, sort_keys=True).encode())
        if self.path.exists():
            self.s = json.loads(self.path.read_text())
            if self.s.get('binding') != binding:
                raise MirrorError('mirror configuration changed')
        else:
            self.s = {'version': 1, 'binding': binding, 'origin_run': config['origin_run'],
                      'mirror_run': config['mirror_run'], 'cursors': [0, 0], 'sources': [],
                      'boundary': False, 'count': 0, 'sha256': digest(b''), 'pending': None,
                      'quiet_terminal_polls': 0, 'complete': False}
            self.save()

    def save(self):
        tmp = self.path.with_suffix('.json.partial')
        with open(tmp, 'w', encoding='utf-8') as f:
            os.chmod(tmp, 0o600)
            json.dump(self.s, f, sort_keys=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)

    def source_data(self):
        out, identities = [], []
        for i, src in enumerate(self.c['sources']):
            p = Path(src['path'])
            with open(p, 'rb') as f:
                st = os.fstat(f.fileno())
                data = f.read()
            ident = {'device': st.st_dev, 'inode': st.st_ino}
            if self.s['sources'] and ident != self.s['sources'][i]['identity']:
                raise MirrorError('source file identity changed')
            if i == 0 and digest(data) != src['sha256']:
                raise MirrorError('immutable source changed')
            cursor = self.s['cursors'][i]
            if len(data) < cursor:
                raise MirrorError('source truncated')
            prefix_sha = digest(data[:cursor])
            if self.s['sources'] and prefix_sha != self.s['sources'][i]['prefix_sha256']:
                raise MirrorError('consumed source prefix changed')
            out.append(data)
            identities.append({'identity': ident, 'prefix_sha256': prefix_sha})
        if not self.s['sources']:
            self.s['sources'] = identities
            self.save()
        return out

    def build(self, data, timestamp, max_lines=200):
        cursors = list(self.s['cursors'])
        boundary = self.s['boundary']
        raw = []
        for i in range(2):
            if i == 1 and not boundary:
                if cursors[0] != len(data[0]):
                    break
                raw.append('--- console recovery: original host -> resumed host; origin run unchanged ---\n')
                boundary = True
            while len(raw) < max_lines:
                end = data[i].find(b'\n', cursors[i])
                if end < 0:
                    break
                line = data[i][cursors[i]:end + 1].decode('utf-8', errors='strict')
                check_secrets(line, self.key)
                raw.append(line)
                cursors[i] = end + 1
            if len(raw) >= max_lines:
                break
            if i == 0 and cursors[0] != len(data[0]):
                raise MirrorError('immutable source ends with an incomplete line')
        lines = [timestamp + ' ' + line for line in raw]
        return lines, cursors, boundary

    def reconcile(self, data):
        pending = self.s['pending']
        if not pending:
            return False
        lines, cursors, boundary = self.build(data, pending['timestamp'], pending['lines'])
        if digest(encoded(canonical(lines))) != pending['sha256'] or len(lines) != pending['lines']:
            raise MirrorError('pending source text changed')
        server = self.api.output()
        n, base = len(server), self.s['count']
        if n < base or n > base + len(lines) or digest(encoded(server[:base])) != self.s['sha256']:
            raise MirrorError('unexpected server console count or committed prefix')
        accepted = n - base
        if server[base:] != canonical(lines[:accepted]):
            raise MirrorError('server console chunk differs from pending transaction')
        if accepted < len(lines):
            committed_prefix = server[:base]
            self.api.append(n, lines[accepted:])
            # An acknowledged POST is still independently verified before committing.
            server = self.api.output()
            expected = committed_prefix + canonical(lines)
            if server != expected:
                if base <= len(server) < len(expected) and server == expected[:len(server)]:
                    raise RetryableError('server console readback has not caught up')
                raise MirrorError('server did not confirm exact appended chunk')
        self.s.update(cursors=cursors, boundary=boundary, count=base + len(lines),
                      sha256=digest(encoded(server)), pending=None, quiet_terminal_polls=0)
        for i in range(2):
            self.s['sources'][i]['prefix_sha256'] = digest(data[i][:cursors[i]])
        self.save()
        return True

    def poll(self):
        if self.s['complete']:
            return False
        data = self.source_data()
        changed = self.reconcile(data)
        lines, _, _ = self.build(data, self.clock())
        if lines:
            # Scan the whole available suffix first: never upload earlier lines from
            # this poll if a credential is present later in the same source suffix.
            for i, source in enumerate(data):
                complete = source[self.s['cursors'][i]:].rsplit(b'\n', 1)[0]
                check_secrets(complete.decode('utf-8'), self.key)
            timestamp = lines[0].split(' ', 1)[0]
            self.s['pending'] = {'timestamp': timestamp, 'sha256': digest(encoded(canonical(lines))), 'lines': len(lines)}
            self.save()
            return self.reconcile(data)
        if changed:
            # A recovered pending transaction just flushed data. Quiet terminal
            # observations must start only on a later unchanged poll.
            return True
        status = None
        if self.c.get('training_status'):
            try:
                status = json.loads(Path(self.c['training_status']).read_text()).get('state')
            except (OSError, ValueError):
                pass
        terminal = status in ('finished', 'failed', 'completed', 'succeeded')
        # A trailing partial line is never silently discarded at completion.
        pending_bytes = any(len(data[i]) != self.s['cursors'][i] for i in range(2))
        self.s['quiet_terminal_polls'] = self.s['quiet_terminal_polls'] + 1 if terminal and not pending_bytes else 0
        if self.s['quiet_terminal_polls'] >= 2:
            self.api.complete()
            self.s['complete'] = True
        else:
            self.api.heartbeat()
        self.save()
        return changed


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', required=True)
    p.add_argument('--once', action='store_true', help='one bounded poll, useful for controlled deployment')
    args = p.parse_args(argv)
    try:
        c = load_config(args.config)
        key = os.environ.get('WANDB_API_KEY')
        if not key:
            auth = netrc.netrc().authenticators('api.wandb.ai')
            key = auth[2] if auth else None
        if not key:
            raise MirrorError('W&B authentication unavailable')
        state_dir = Path(c['state_dir'])
        state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        with open(state_dir / 'console_mirror.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            api = WandbAPI(c, key)
            mirror = Mirror(c, api, key)
            # Validate sources and credential filters before creating any remote run.
            source = mirror.source_data()
            for data in source:
                check_secrets(data.rsplit(b'\n', 1)[0].decode('utf-8'), key)
            api.ensure_run()
            while True:
                try:
                    changed = mirror.poll()
                except (urllib.error.URLError, TimeoutError, ConnectionError, RetryableError):
                    # A pending cursor is intentionally retained for exact reconciliation.
                    print(json.dumps({'event': 'console_mirror_retryable_transport_error'}), flush=True)
                    if args.once:
                        return 2
                    changed = False
                print(json.dumps({'event': 'console_mirror_status', 'mirror_run': c['mirror_run'],
                                  'lines': mirror.s['count'], 'complete': mirror.s['complete']}), flush=True)
                if args.once or mirror.s['complete']:
                    return 0
                if not changed:
                    time.sleep(c['poll_interval_seconds'])
    except Exception as exc:
        # Do not print exception text: HTTP/config exceptions may contain secrets.
        print(json.dumps({'event': 'console_mirror_failed', 'error_type': type(exc).__name__}), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
