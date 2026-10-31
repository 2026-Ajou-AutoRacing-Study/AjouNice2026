import hashlib
import json
import re
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import check_perception_context as context
from check_perception_context import anchors, references, slug, ROOT, audit


class ContextTests(unittest.TestCase):
    def test_unicode_heading_and_punctuation(self):
        self.assertEqual(slug('현재 기준: Base5/3D'), '현재-기준-base53d')

    def test_explicit_and_duplicate_anchors(self):
        result=anchors('<a id="runtime"></a>\n## 같은 제목\n## 같은 제목\n')
        self.assertEqual(result, {'runtime','같은-제목','같은-제목-1'})

    def test_fenced_code_does_not_create_headings_or_anchors(self):
        result=anchors('## 실제 절\n```bash\n# 주석\n<a id="fake"></a>\n```\n')
        self.assertEqual(result, {'실제-절'})

    def test_relative_fragment_and_encoded_path(self):
        path=Path('/repo/docs/perception/README.md')
        result=list(references(path,'[link](../bevfusion/My%20Doc.md#runtime)'))
        self.assertEqual(result, [(Path('/repo/docs/bevfusion/My Doc.md'),'runtime')])

    def test_external_links_do_not_request_network(self):
        result=list(references(Path('/repo/README.md'),'[remote](https://example.com/x#y)'))
        self.assertEqual(result, [])

    def test_fragment_only_refers_to_same_file(self):
        path=Path('/repo/CURRENT.md')
        self.assertEqual(list(references(path,'[runtime](#runtime)')),[(path,'runtime')])

    def test_fenced_examples_are_not_mandatory_dependencies(self):
        text='```md\n[old](missing.md)\n```\n[current](CURRENT.md)'
        result=list(references(Path('/repo/README.md'),text))
        self.assertEqual(result, [(Path('/repo/CURRENT.md'),'')])

    def test_actual_context_graph_archives_and_budgets(self):
        report=audit(ROOT)
        self.assertEqual(report['errors'], [])
        self.assertEqual(report['preserved_originals'],13)
        self.assertEqual(report['verified_snapshot_sha256'],13)
        self.assertEqual(report['verified_git_snapshots'],13)
        self.assertLess(report['same_large_agent_lines_after'],report['archived_large_agent_lines_before']/4)
        self.assertGreater(report['checked_incoming_links'],0)

    def test_active_context_does_not_force_dated_evidence_or_archives(self):
        self.assertFalse(any('/legacy/' in name or '/evidence/' in name for name in context.ACTIVE))
        self.assertFalse(any(re.search(r'/\d{6}_', name) for name in context.ACTIVE))
        for name in ('AGENTS.md', 'docs/README.md', 'kcity_map/README.md',
                     'docs/bevfusion/BASE5_BEVFUSION_PRODUCTION_HANDOFF.md',
                     'docs/bevfusion/GT_EVALUATION_COORDINATE_CONTRACT.md'):
            self.assertIn(name, context.ACTIVE)

    def test_current_contracts_replace_old_completion_markers(self):
        model = context.CONTRACTS['model'][1]
        dataset = context.CONTRACTS['dataset'][1]
        for marker in ('E12', 'camera TRT', 'E12 eager', '512×1056', '0.40/0.10/0.10'):
            self.assertIn(marker, model)
        for marker in ('30,750', '26,136', '4,614', '3클래스', 'Q90 JPEG'):
            self.assertIn(marker, dataset)
        self.assertNotIn('flexible_v140', dataset)
        self.assertNotIn('fresh_suite_final_v142', dataset)


class AuditFailureTests(unittest.TestCase):
    """Small pure fixtures exercise failure reporting without the live repo graph."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.original = b'# Historical\n<a id="old-anchor"></a>\n'
        self.write('CURRENT.md', '# Current\nE12 camera TRT E12 eager\n')
        self.write('AGENTS.md', '# Router\n[model](CURRENT.md)\n')
        self.write('incoming.md', '[current](CURRENT.md#current)\n')
        self.write('legacy/old.md', self.original)
        self.snapshot = 'docs/perception/legacy/context_261012/old.txt'
        self.write(self.snapshot, self.original)
        self.manifest_path = 'docs/perception/legacy/context_261012/manifest.json'
        self.manifest = dict(source_commit='abc1234', files=[dict(
            original_path='old.md', archive_path=self.snapshot,
            sha256=hashlib.sha256(self.original).hexdigest())])
        self.write_manifest()
        constants = mock.patch.multiple(context,
            AGENTS=('AGENTS.md',), ACTIVE=('AGENTS.md', 'CURRENT.md'),
            BUDGETS={'AGENTS.md': 45, 'CURRENT.md': 125},
            COMPATIBILITY={'old.md': 'legacy/old.md'},
            CONTRACTS={'model': ('CURRENT.md', ('E12', 'camera TRT', 'E12 eager'))})
        constants.start(); self.addCleanup(constants.stop)
        git_patch = mock.patch.object(context.subprocess, 'run', side_effect=self.git_run)
        self.git = git_patch.start(); self.addCleanup(git_patch.stop)

    def write(self, name, value):
        path = self.root/name
        path.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(value, bytes):
            path.write_bytes(value)
        else:
            path.write_text(value)
        return path

    def write_manifest(self):
        self.write(self.manifest_path, json.dumps(self.manifest))

    def git_run(self, command, **kwargs):
        if command[1] == 'show':
            return subprocess.CompletedProcess(command, 0, self.original, b'')
        return subprocess.CompletedProcess(command, 0, b'AGENTS.md\nCURRENT.md\nincoming.md\n', b'')

    def errors(self):
        return '\n'.join(audit(self.root)['errors'])

    def test_moved_compatibility_archive_preserves_old_anchors(self):
        report = audit(self.root)
        self.assertEqual(report['errors'], [])
        self.assertEqual(report['compatibility_paths'], {'old.md': 'legacy/old.md'})
        self.assertEqual(report['verified_snapshot_sha256'], 1)
        self.assertEqual(report['verified_git_snapshots'], 1)
        self.assertEqual(report['checked_incoming_links'], 1)

    def test_retained_stub_takes_precedence_and_requires_historical_anchors(self):
        self.write('old.md', '# Compatibility\n<a id="old-anchor"></a>\n<a id="historical"></a>\n')
        report = audit(self.root)
        self.assertEqual(report['errors'], [])
        self.assertEqual(report['compatibility_paths']['old.md'], 'old.md')
        self.write('old.md', '# Compatibility without historical anchors\n')
        self.assertIn('lost historical anchors', self.errors())

    def test_missing_compatibility_destination_is_reported(self):
        (self.root/'legacy/old.md').unlink()
        self.assertIn('Missing compatibility destination', self.errors())

    def test_missing_active_and_contract_file_does_not_crash(self):
        (self.root/'CURRENT.md').unlink()
        errors = self.errors()
        self.assertIn('Missing active context: CURRENT.md', errors)
        self.assertIn('Cannot read context CURRENT.md', errors)

    def test_missing_snapshot_manifest_does_not_crash(self):
        (self.root/self.manifest_path).unlink()
        self.assertIn('Cannot read context '+self.manifest_path, self.errors())

    def test_malformed_snapshot_manifest_does_not_crash(self):
        self.write(self.manifest_path, '{broken')
        self.assertIn('Invalid snapshot manifest', self.errors())

    def test_manifest_shape_and_records_are_validated(self):
        self.write(self.manifest_path, '[]')
        self.assertIn('object required', self.errors())
        self.manifest['files'] = 'not a list'
        self.write_manifest()
        self.assertIn('files must be a list', self.errors())
        self.manifest['files'] = [None, dict(original_path='../outside.md', archive_path=self.snapshot)]
        self.write_manifest()
        errors = self.errors()
        self.assertIn('Snapshot record 0: object required', errors)
        self.assertIn('path escapes repository', errors)
        self.assertIn('malformed SHA256', errors)

    def test_duplicate_snapshot_original_is_reported(self):
        self.manifest['files'].append(dict(self.manifest['files'][0]))
        self.write_manifest()
        self.assertIn('duplicate original_path', self.errors())

    def test_missing_original_snapshot_does_not_crash(self):
        (self.root/self.snapshot).unlink()
        self.assertIn('Missing original snapshot', self.errors())

    def test_non_utf8_snapshot_is_reported(self):
        self.write(self.snapshot, b'\xff\xfe')
        self.assertIn('Invalid original snapshot text', self.errors())

    def test_missing_snapshot_record_paths_are_reported(self):
        self.manifest['files'] = [dict(sha256='0'*64)]
        self.write_manifest()
        self.assertIn('missing relative path', self.errors())

    def test_changed_snapshot_keeps_sha_and_git_validation(self):
        self.write(self.snapshot, b'changed original\n')
        errors = self.errors()
        self.assertIn('Changed original snapshot', errors)
        self.assertIn('Snapshot differs from source commit', errors)

    def test_unavailable_git_reports_errors_without_exception(self):
        self.git.side_effect = FileNotFoundError('git unavailable')
        errors = self.errors()
        self.assertIn('cannot run Git', errors)

    def test_bad_source_revision_is_reported_without_running_git_show(self):
        self.manifest['source_commit'] = '--not-a-revision'
        self.write_manifest()
        self.assertIn('hexadecimal source_commit required', self.errors())
        self.assertFalse(any(call.args[0][1] == 'show' for call in self.git.call_args_list))

    def test_git_show_failure_is_reported(self):
        def failed(command, **kwargs):
            if command[1] == 'show':
                return subprocess.CompletedProcess(command, 128, b'', b'unknown revision')
            return self.git_run(command, **kwargs)
        self.git.side_effect = failed
        self.assertIn('Git failed (128): unknown revision', self.errors())

    def test_missing_contract_marker_is_reported(self):
        self.write('CURRENT.md', '# Current\nE25 historical default\n')
        self.assertIn('Missing model contract marker: E12', self.errors())

    def test_line_budget_is_enforced_for_current_router(self):
        self.write('AGENTS.md', '# Router\n' * 46)
        self.assertIn('exceeds 45-line current/index budget', self.errors())

    def test_malformed_markdown_link_is_reported(self):
        self.write('CURRENT.md', '# Current\nE12 camera TRT E12 eager\n[bad](http://[)\n')
        self.assertIn('malformed Markdown link', self.errors())

    def test_non_utf8_active_context_is_reported(self):
        self.write('CURRENT.md', b'\xff\xfe')
        self.assertIn('Cannot read context CURRENT.md', self.errors())


if __name__=='__main__':unittest.main()
