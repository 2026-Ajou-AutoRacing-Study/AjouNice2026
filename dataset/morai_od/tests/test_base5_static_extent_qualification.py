import copy
from collections import Counter
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_static_mission_extent as extent
import prepare_base5_static_extent_qualification as qualification


def metadata():
    result = {}
    index = 0
    for slot_index, (name, split, phase) in enumerate(qualification.SLOTS):
        count = 38 if slot_index < 8 else 37
        for ordinal in range(count):
            pool = qualification.POOLS[(ordinal+slot_index) % len(qualification.POOLS)]
            result['scene_'+str(index).zfill(4)] = dict(split=split,
                spec=dict(family='obstacle', mission=True, avoidance_phase=phase,
                    stationary_mission_pool=pool, static_mission_extent=dict(stratum=name)),
                runtime_actors=[dict(data_id=index, name=pool+'_'+str(index))])
            index += 1
    return result


class QualificationTests(unittest.TestCase):
    def test_fixed_16_cells_cover_all_pools_without_outcome_selection(self):
        source = metadata()
        before = copy.deepcopy(source)
        selected = qualification.select(source)
        self.assertEqual(source, before)
        self.assertEqual(selected, qualification.select(dict(reversed(list(source.items())))))
        self.assertEqual(len(selected), 16)
        self.assertEqual(len({row['id'] for row in selected}), 16)
        self.assertEqual({row['primary_pool'] for row in selected}, set(qualification.POOLS))
        self.assertEqual(Counter((row['route_stratum'], row['split']) for row in selected),
            Counter({('before', 'train'):4, ('intersection', 'val'):4, ('after', 'train'):4, ('after', 'val'):4}))
        self.assertEqual(Counter(row['phase'] for row in selected), Counter({phase:4 for phase in qualification.PHASES}))

    def test_partial_source_missing_cells_or_pools_are_rejected(self):
        source = metadata()
        del source['scene_0000']
        with self.assertRaises(extent.ExtentContractError):
            qualification.select(source)
        source = metadata()
        for meta in source.values():
            if meta['spec']['stationary_mission_pool'] == 'animal':
                meta['spec']['stationary_mission_pool'] = 'person'
        with self.assertRaisesRegex(extent.ExtentContractError, 'nine'):
            qualification.select(source)

    def test_foreign_family_or_unknown_pool_is_rejected(self):
        for key, value in (('family', 'highway'), ('stationary_mission_pool', 'unknown')):
            source = metadata()
            source['scene_0000']['spec'][key] = value
            with self.assertRaises(extent.ExtentContractError):
                qualification.select(source)

    def test_output_guards_happen_before_reading_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with self.assertRaises(FileExistsError):
                qualification.prepare(root/'missing', root)
            with self.assertRaisesRegex(ValueError, 'outside'):
                qualification.prepare(root/'manifest.json', root/'child')


if __name__ == '__main__':
    unittest.main()
