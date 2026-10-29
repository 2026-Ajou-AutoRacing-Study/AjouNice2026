import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from planning_scenario_validator.scenario_order import ordered_sources


class ScenarioOrderTests(unittest.TestCase):
    def setUp(self):
        self.sources = [Path('/scenes/a.json'), Path('/scenes/b.json'), Path('/scenes/c.json')]

    def test_default_order_is_unchanged(self):
        self.assertEqual(ordered_sources(self.sources), self.sources)

    def test_explicit_permutation_preserves_original_filenames(self):
        result = ordered_sources(self.sources, ['c.json', 'a.json', 'b.json'])
        self.assertEqual(result, [self.sources[2], self.sources[0], self.sources[1]])
        self.assertEqual(self.sources[0].name, 'a.json')

    def test_duplicates_are_rejected(self):
        with self.assertRaises(ValueError):
            ordered_sources(self.sources, ['a.json', 'a.json', 'c.json'])

    def test_missing_or_foreign_sources_are_rejected(self):
        for order in (['a.json'], ['a.json', 'b.json', '../c.json']):
            with self.subTest(order=order), self.assertRaises(ValueError):
                ordered_sources(self.sources, order)

    def test_invalid_order_types_are_rejected(self):
        for order in ('a.json', [1, 2, 3]):
            with self.subTest(order=order), self.assertRaises(ValueError):
                ordered_sources(self.sources, order)

    def test_snapshot_copy_retains_execution_order(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, snapshot = root / 'source', root / 'snapshot'
            source.mkdir(); snapshot.mkdir()
            for path in self.sources:
                (source / path.name).write_text('{}')
                shutil.copy2(source / path.name, snapshot / path.name)
            order = ['b.json', 'c.json', 'a.json']
            original = ordered_sources(sorted(source.glob('*.json')), order)
            frozen = ordered_sources(sorted(snapshot.glob('*.json')), order)
            self.assertEqual([path.name for path in original], [path.name for path in frozen])


if __name__ == '__main__':
    unittest.main()
