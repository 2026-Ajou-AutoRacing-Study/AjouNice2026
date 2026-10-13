import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import atomic_json, sha256
from base5_intersection_ground import POLICY, load_lookup
from merge_base5_intersection_ground import merge


class ReleaseTest(unittest.TestCase):
    def fixture(self, root, key, accepted=2):
        evidence = root/(key+'_report.json')
        atomic_json(evidence, dict(status='COMPLETE', captures=2, accepted=accepted,
                                  validated_profiles=[key], files=[]))
        lookup = root/(key+'.json')
        atomic_json(lookup, dict(policy=POLICY, status='LIVE_VALIDATED', sources=[],
            profiles={key:dict(asset=key)},
            live_evidence=dict(path=str(evidence), sha256=sha256(evidence), validated_profiles=[key])))
        return lookup

    def test_merge_preserves_parent_hashes_and_profile_union(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            inputs = [self.fixture(root, 'a'), self.fixture(root, 'b')]
            path = merge(inputs, root/'release')
            data = load_lookup(path)
            self.assertEqual(set(data['profiles']), {'a', 'b'})
            evidence = json.loads(Path(data['live_evidence']['path']).read_text())
            self.assertEqual(evidence['captures'], 4)
            self.assertEqual(len(data['sources']), 4)
            atomic_json(inputs[0], {})
            with self.assertRaisesRegex(ValueError, 'source hash'):
                load_lookup(path)

    def test_partial_live_report_cannot_be_hidden_by_merge(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            with self.assertRaisesRegex(ValueError, 'failed validation'):
                merge([self.fixture(root, 'a', accepted=1)], root/'release')

    def test_duplicate_profile_is_ambiguous(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d); source = self.fixture(root, 'a')
            with self.assertRaisesRegex(ValueError, 'duplicate profile'):
                merge([source, source], root/'release')


if __name__ == '__main__':
    unittest.main()
