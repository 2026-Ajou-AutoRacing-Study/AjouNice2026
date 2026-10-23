import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('console_mirror', Path(__file__).with_name('base5_wandb_console_mirror.py'))
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


class API:
    def __init__(self):
        self.lines = []
        self.posts = []
        self.fail_after = None
        self.completed = False
        self.heartbeats = 0

    def output(self):
        return list(self.lines)

    def append(self, offset, lines):
        self.posts.append((offset, list(lines)))
        assert offset == len(self.lines), 'must never overwrite'
        if self.fail_after is not None:
            self.lines.extend(m.canonical(lines[:self.fail_after]))
            self.fail_after = None
            raise TimeoutError('ambiguous response')
        self.lines.extend(m.canonical(lines))

    def heartbeat(self):
        self.heartbeats += 1

    def complete(self):
        self.completed = True


class MirrorTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.old = self.root / 'old.log'
        self.new = self.root / 'new.log'
        self.old.write_bytes(b'old first\nold second\n')
        self.new.write_bytes(b'new first\npartial')
        self.c = {'allow_run_create': True, 'entity': 'team', 'project': 'project',
                  'origin_run': 'original', 'mirror_run': 'separate-console',
                  'display_name': 'console only', 'state_dir': str(self.root / 'state'),
                  'poll_interval_seconds': 20,
                  'sources': [{'path': str(self.old), 'immutable': True,
                               'sha256': hashlib.sha256(self.old.read_bytes()).hexdigest()},
                              {'path': str(self.new), 'immutable': False}]}
        self.api = API()

    def mirror(self):
        return m.Mirror(self.c, self.api, 'secret-key-never-log', lambda: '2026-10-22T09:00:00Z')

    def append_new(self, data):
        with open(self.new, 'ab') as f:
            f.write(data)

    def test_source_growth_partial_line_and_order(self):
        mirror = self.mirror()
        mirror.poll()
        self.assertEqual(len(self.api.lines), 4)
        self.assertIn('old first', self.api.lines[0])
        self.assertIn('original host -> resumed host', self.api.lines[2])
        self.assertIn('new first', self.api.lines[3])
        self.assertFalse(any('partial' in line for line in self.api.lines))
        self.append_new(b' done\n')
        mirror.poll()
        self.assertIn('partial done', self.api.lines[-1])

    def test_idempotent_restart(self):
        self.mirror().poll()
        before = list(self.api.lines)
        self.mirror().poll()
        self.assertEqual(self.api.lines, before)
        self.assertEqual(len(self.api.posts), 1)

    def test_ambiguous_partial_response_reconciles_without_overwrite(self):
        self.api.fail_after = 2
        with self.assertRaises(TimeoutError):
            self.mirror().poll()
        self.append_new(b' done\nnew second\n')
        self.mirror().poll()
        self.assertEqual(self.api.posts[1][0], 2)
        self.assertEqual(len([l for l in self.api.lines if 'old first' in l]), 1)
        self.assertIn('new second', self.api.lines[-1])

    def test_response_lost_after_full_commit(self):
        self.api.fail_after = 999
        with self.assertRaises(TimeoutError):
            self.mirror().poll()
        self.mirror().poll()
        self.assertEqual(len(self.api.posts), 1)

    def test_acknowledged_post_delayed_readback_keeps_pending(self):
        mirror = self.mirror()
        original = self.api.output
        counter = [0]
        def delayed():
            counter[0] += 1
            return [] if counter[0] <= 2 else original()
        self.api.output = delayed
        with self.assertRaises(m.RetryableError):
            mirror.poll()
        self.assertIsNotNone(mirror.s['pending'])
        self.api.output = original
        self.mirror().poll()
        self.assertEqual(len(self.api.posts), 1)

    def test_refuse_server_count_or_prefix_change(self):
        mirror = self.mirror()
        mirror.poll()
        self.append_new(b' done\n')
        self.api.lines[0] = 'unexpected writer\n'
        with self.assertRaises(m.MirrorError):
            mirror.poll()
        self.assertEqual(len(self.api.posts), 1)

    def test_refuse_prefix_change_after_post_before_readback(self):
        mirror = self.mirror()
        mirror.poll()
        self.append_new(b' done\n')
        append = self.api.append
        def corrupt(offset, lines):
            append(offset, lines)
            self.api.lines[0] = 'unexpected concurrent mutation\n'
        self.api.append = corrupt
        with self.assertRaises(m.MirrorError):
            mirror.poll()
        self.assertIsNotNone(mirror.s['pending'])

    def test_secret_fail_closed_before_any_post(self):
        self.append_new(b'\nWANDB_API_KEY=not-a-real-key-1234567890\n')
        with self.assertRaises(m.MirrorError):
            self.mirror().poll()
        self.assertEqual(self.api.posts, [])
        self.assertNotIn('not-a-real', (self.root / 'state/console_mirror_state.json').read_text())

    def test_exact_in_memory_key_is_never_uploaded(self):
        self.append_new(b'\nsecret-key-never-log\n')
        with self.assertRaises(m.MirrorError):
            self.mirror().poll()
        self.assertEqual(self.api.posts, [])

    def test_original_mutation_truncation_and_inode_replacement_refused(self):
        for kind in ('old', 'truncate', 'replace'):
            with self.subTest(kind=kind):
                self.c['state_dir'] = str(self.root / ('state-' + kind))
                mirror = self.mirror()
                mirror.poll()
                old, new = self.old.read_bytes(), self.new.read_bytes()
                if kind == 'old':
                    self.old.write_bytes(b'changed\n')
                elif kind == 'truncate':
                    self.new.write_bytes(b'')
                else:
                    replacement = self.root / 'replacement'
                    replacement.write_bytes(new)
                    replacement.replace(self.new)
                with self.assertRaises(m.MirrorError):
                    mirror.poll()
                self.old.write_bytes(old)
                self.new.write_bytes(new)
                self.api = API()

    def test_complete_only_two_quiet_terminal_polls(self):
        status = self.root / 'status.json'
        self.c['training_status'] = str(status)
        mirror = self.mirror()
        mirror.poll()
        status.write_text(json.dumps({'state': 'finished'}))
        mirror.poll()
        mirror.poll()
        self.assertFalse(self.api.completed, 'partial source line still exists')
        self.append_new(b' done\n')
        mirror.poll()
        mirror.poll()
        self.assertFalse(self.api.completed)
        mirror.poll()
        self.assertTrue(self.api.completed)

    def test_config_authority_same_run_and_unknown_keys(self):
        p = self.root / 'config.json'
        for change in ({'allow_run_create': False}, {'mirror_run': 'original'}, {'api_key': 'forbidden'}):
            c = copy.deepcopy(self.c)
            c.update(change)
            p.write_text(json.dumps(c))
            with self.assertRaises(m.MirrorError):
                m.load_config(p)

    def test_protocol_targets_auxiliary_only_and_no_metrics(self):
        api = m.WandbAPI(self.c, 'secret-key-never-log')
        calls = []
        api.request = lambda path, payload=None, **kwargs: calls.append((path, payload)) or b'{}'
        api.append(8, ['2026-10-22T09:00:00Z hello\n'])
        api.heartbeat()
        api.complete()
        for path, body in calls:
            self.assertIn('/separate-console/file_stream', path)
            self.assertNotIn('/original/', path)
            self.assertNotIn('history', str(body))
        self.assertEqual(calls[0][1]['files']['output.log']['offset'], 8)
        self.assertFalse(calls[0][1]['complete'])

    def test_create_new_run_only_and_missing_run_graphql_error(self):
        api = m.WandbAPI(self.c, 'secret-key-never-log')
        calls = []
        def request(path, payload=None, **kwargs):
            calls.append((path, payload))
            if len(calls) == 1:
                return json.dumps({'errors': [{'message': 'Could not find run separate-console'}]}).encode()
            return b'{"data":{"upsertBucket":{"bucket":{"name":"separate-console"}}}}'
        api.request = request
        api.ensure_run()
        inp = calls[1][1]['variables']['input']
        self.assertEqual(inp['name'], 'separate-console')
        self.assertEqual(inp['jobType'], 'console_recovery')
        self.assertEqual(set(json.loads(inp['config'])), {'origin_run_url'})
        self.assertNotEqual(inp['name'], self.c['origin_run'])

    def test_graphql_improved_pagination_and_timestamp_canonicalization(self):
        api = m.WandbAPI(self.c, 'secret-key-never-log')
        calls = []
        def graphql(query, variables):
            calls.append((query, variables))
            first = variables['after'] is None
            edge = {'cursor': 'a' if first else 'b',
                    'node': {'number': 0 if first else 1, 'timestamp': 1792655041.0,
                             'line': 'first' if first else 'second'}}
            return {'project': {'run': {'logLineCount': 0, 'logLines': {
                'edges': [edge], 'pageInfo': {'hasNextPage': first, 'endCursor': edge['cursor']}}}}}
        api.graphql = graphql
        self.assertEqual(api.output(), ['first\n', 'second\n'])
        self.assertIn('useImprovedPagination:true', calls[0][0])
        self.assertEqual(calls[1][1]['after'], 'a')
        self.assertEqual(m.canonical(['2026-10-22T09:00:00.000000Z first\n']), ['first\n'])
        self.assertEqual(m.digest(m.encoded(m.canonical(['2026-10-22T09:00:00Z first\n']))),
                         m.digest(m.encoded(['first\n'])))

    def test_graphql_rejects_missing_or_reordered_row_number(self):
        api = m.WandbAPI(self.c, 'secret-key-never-log')
        api.graphql = lambda *a, **kw: {'project': {'run': {'logLineCount': 0, 'logLines': {
            'edges': [{'cursor': 'b', 'node': {'number': 1, 'timestamp': 0, 'line': 'out of order'}}],
            'pageInfo': {'hasNextPage': False, 'endCursor': 'b'}}}}}
        with self.assertRaises(m.MirrorError):
            api.output()

    def test_recovered_final_transaction_does_not_count_as_quiet_poll(self):
        self.new.write_bytes(b'complete\n')
        status = self.root / 'status.json'
        status.write_text(json.dumps({'state': 'finished'}))
        self.c['training_status'] = str(status)
        self.api.fail_after = 999
        with self.assertRaises(TimeoutError):
            self.mirror().poll()
        mirror = self.mirror()
        mirror.poll()
        self.assertEqual(mirror.s['quiet_terminal_polls'], 0)
        mirror.poll()
        self.assertFalse(self.api.completed)
        mirror.poll()
        self.assertTrue(self.api.completed)

    def test_preserve_original_timestamp_in_input_text(self):
        self.new.write_bytes(b'2026-10-22T08:00:00Z actual original console text\n')
        mirror = self.mirror()
        mirror.poll()
        self.assertEqual(self.api.lines[-1], '2026-10-22T08:00:00Z actual original console text\n')
        self.append_new(b'next\n')
        mirror.poll()
        self.assertEqual(self.api.lines[-1], 'next\n')


if __name__ == '__main__':
    unittest.main()
