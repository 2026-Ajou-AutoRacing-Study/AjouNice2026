import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_front_release_inputs as pins
import launch_base5_obstacle_support as launch


def write(path, value):
    path.write_text(json.dumps(value))
    return path


class RuntimePinsTests(unittest.TestCase):
    def test_import_and_literal_subprocess_closure_without_external_import(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'main.py').write_text("import child\nimport torch\nPath(__file__).with_name('worker.py')\n")
            (root / 'child.py').write_text('from leaf import value\n')
            (root / 'leaf.py').write_text('value = 1\n')
            (root / 'worker.py').write_text('raise RuntimeError("must never execute")\n')
            result = pins.collect_runtime(root, ['main.py'])
            self.assertEqual({Path(path).name for path in result},
                             {'main.py', 'child.py', 'leaf.py', 'worker.py'})

    def test_nested_mutation_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'main.py').write_text('import child\n')
            (root / 'child.py').write_text('value = 1\n')
            result = pins.collect_runtime(root, ['main.py'])
            (root / 'child.py').write_text('value = 2\n')
            with self.assertRaisesRegex(ValueError, 'changed'):
                pins.verify_hashes(result)

    def test_missing_entrypoint_import_and_literal_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for content in ("import base5_missing\n", "Path(__file__).with_name('missing.py')\n"):
                (root / 'main.py').write_text(content)
                with self.assertRaisesRegex(ValueError, 'Missing'):
                    pins.collect_runtime(root, ['main.py'])
            with self.assertRaisesRegex(ValueError, 'Missing'):
                pins.collect_runtime(root, ['missing.py'])

    def test_syntax_and_path_escape_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'main.py').write_text('def broken(:\n')
            with self.assertRaisesRegex(ValueError, 'Invalid'):
                pins.collect_runtime(root, ['main.py'])
            (root / 'main.py').write_text("Path(__file__).with_name('../escape.py')\n")
            with self.assertRaisesRegex(ValueError, 'escaped'):
                pins.collect_runtime(root, ['main.py'])


class InputPinsTests(unittest.TestCase):
    def fixture(self, root):
        reference = write(root / 'reference.json', {'value': 1})
        spec = {'seed': 17}
        scenario = write(root / 'scene.json', {'ego': 1})
        meta = write(root / 'meta.json', {'spec': spec})
        frozen = write(root / 'frozen.json', [spec])
        manifest = write(root / 'manifest.json', {
            'count': 1, 'config': {
                'reference': {'path': str(reference), 'sha256': pins.file_sha256(reference)},
                'frozen_specs_path': str(frozen), 'frozen_specs_sha256': pins.file_sha256(frozen)},
            'scenes': [{'id': 'one', 'scenario': str(scenario),
                        'metadata': str(meta), 'scenario_sha256': pins.file_sha256(scenario)}]})
        return manifest, scenario, meta, reference, frozen

    def test_complete_manifest_and_declared_references_are_pinned(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, *paths = self.fixture(root)
            original = {str(path): path.read_bytes() for path in (manifest, *paths)}
            actual = pins.collect_inputs(manifest, 1, {})
            self.assertEqual(set(actual), set(original))
            self.assertEqual(original, {str(path): path.read_bytes() for path in (manifest, *paths)})

    def test_metadata_and_data_mutations_are_not_silently_refrozen(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, scenario, meta, reference, frozen = self.fixture(root)
            actual = pins.collect_inputs(manifest, 1, {})
            meta.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'changed'):
                pins.verify_hashes(actual)
            with self.assertRaisesRegex(ValueError, 'Frozen specs'):
                pins.collect_inputs(manifest, 1, {})
            write(meta, {'spec': {'seed': 17}})
            reference.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'Declared'):
                pins.collect_inputs(manifest, 1, {})

    def test_missing_and_invalid_declared_digests_fail(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, scenario, meta, reference, frozen = self.fixture(root)
            doc = json.loads(manifest.read_text())
            doc['scenes'][0]['scenario_sha256'] = 'bad'
            write(manifest, doc)
            with self.assertRaisesRegex(ValueError, 'Declared'):
                pins.collect_inputs(manifest, 1, {})
            doc['scenes'][0].pop('scenario_sha256')
            write(manifest, doc)
            with self.assertRaisesRegex(ValueError, 'lacks scenario'):
                pins.collect_inputs(manifest, 1, {})

    def test_count_duplicate_identity_path_and_payload_fail(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, *_ = self.fixture(root)
            with self.assertRaisesRegex(ValueError, 'count'):
                pins.collect_inputs(manifest, 5000, {})
            doc = json.loads(manifest.read_text())
            doc['count'] = 2
            doc['scenes'].append(dict(doc['scenes'][0]))
            write(manifest, doc)
            with self.assertRaisesRegex(ValueError, 'identity'):
                pins.collect_inputs(manifest, 2, {})
            doc['scenes'][1]['id'] = 'two'
            write(manifest, doc)
            with self.assertRaisesRegex(ValueError, 'repeats'):
                pins.collect_inputs(manifest, 2, {})

    def test_nested_source_reference_hash_and_missing_file_are_checked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, scenario, meta, reference, frozen = self.fixture(root)
            child = write(root / 'child.json', {'value': 3})
            write(reference, {'sources': [{'path': str(child), 'sha256': pins.file_sha256(child)}]})
            doc = json.loads(manifest.read_text())
            doc['config']['reference']['sha256'] = pins.file_sha256(reference)
            write(manifest, doc)
            actual = pins.collect_inputs(manifest, 1, {})
            self.assertIn(str(child), actual)
            child.unlink()
            with self.assertRaisesRegex(ValueError, 'Missing'):
                pins.collect_inputs(manifest, 1, {})

    def test_source_manifest_reference_and_declared_metadata_hash_are_checked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, scenario, meta, reference, frozen = self.fixture(root)
            doc = json.loads(manifest.read_text())
            doc['config']['policy'] = {
                'source_manifest': str(reference),
                'source_manifest_sha256': pins.file_sha256(reference)}
            doc['scenes'][0]['metadata_sha256'] = '0' * 64
            write(manifest, doc)
            with self.assertRaisesRegex(ValueError, 'Declared'):
                pins.collect_inputs(manifest, 1, {})
            doc['scenes'][0]['metadata_sha256'] = pins.file_sha256(meta)
            doc['config']['policy']['source_manifest_sha256'] = '0' * 64
            write(manifest, doc)
            with self.assertRaisesRegex(ValueError, 'Declared'):
                pins.collect_inputs(manifest, 1, {})

    def test_non_reference_path_in_lookup_report_is_not_mistaken_for_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, scenario, meta, reference, frozen = self.fixture(root)
            write(reference, {'route': {'path': ['link1', 'link2']},
                              'historical_display': {'path': '/not/an/input'}})
            doc = json.loads(manifest.read_text())
            doc['config']['reference']['sha256'] = pins.file_sha256(reference)
            write(manifest, doc)
            pins.collect_inputs(manifest, 1, {})

    def test_default_reference_expected_hash_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, scenario, meta, reference, frozen = self.fixture(root)
            with self.assertRaisesRegex(ValueError, 'Declared'):
                pins.collect_inputs(manifest, 1, {reference: '0' * 64})

    def test_pin_mapping_requires_absolute_nonempty_sha256(self):
        for value in ({}, [], {'relative': '0' * 64}, {'/tmp/not-read': 'invalid'}):
            with self.assertRaises(ValueError):
                pins.verify_hashes(value)


class LauncherInputPinsTests(unittest.TestCase):
    def test_optional_new_input_pins_checked_before_prepare_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = write(root / 'manifest.json', {'count': 5000, 'config': {'obstacle_support_policy': True}})
            source = root / 'runtime.py'; source.write_text('value = 1\n')
            data = root / 'data.json'; data.write_text('{}')
            release = write(root / 'release.json', {
                'schema': 'base5_obstacle_support_release_v1',
                'qualified_environments': ['sunny13'],
                'runtime_hashes': {str(source): pins.file_sha256(source)},
                'input_hashes': {str(data): pins.file_sha256(data)},
                'environments': {'sunny13': {'manifest': str(manifest),
                                             'manifest_sha256': pins.file_sha256(manifest)}}})
            launch.preflight(release, 'sunny13')
            data.write_text('changed')
            with self.assertRaisesRegex(ValueError, 'Frozen support input changed'):
                launch.preflight(release, 'sunny13')
            doc = json.loads(release.read_text()); doc.pop('input_hashes')
            write(release, doc)
            launch.preflight(release, 'sunny13')


if __name__ == '__main__':
    unittest.main()
