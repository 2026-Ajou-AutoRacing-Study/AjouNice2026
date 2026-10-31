import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_asset_scale as scale
import prepare_base5_deferred_scale_policy as defer


class DeferredScalePolicyTests(unittest.TestCase):
    def setUp(self):
        self.source = scale.candidate_policy([2, 0, 1])
        assets = ('CargoBox', 'WoodBox', 'RedBarrel', 'YellowBarrel', 'PE_Drum',
                  'PE_Firewall_Orange', 'Steel_Barricade', 'Concrete_Barricade01')
        self.source['assets'] = {name: self.source['assets'][name] for name in assets}
        self.source['assets']['CargoBox']['profiles'].pop('sample_2_3_2')
        self.source['assets']['RedBarrel']['profiles']['iso125'] = self.source['assets']['RedBarrel']['profiles'].pop('iso120')
        self.cases = [dict(id='first', profile_id='iso125'), dict(id='second', profile_id='iso150')]
        self.evidence = dict(source_policy=dict(path='/synthetic/policy.json', sha256='1'*64),
                             deferral_report=dict(path='/synthetic/report.json', sha256='2'*64))

    def test_only_red_variants_removed_and_retained_profiles_are_exact(self):
        original = copy.deepcopy(self.source)
        actual = defer.derive(self.source, self.cases, self.evidence)
        self.assertEqual(self.source, original)
        self.assertNotIn('RedBarrel', actual['assets'])
        self.assertEqual(len(actual['assets']), 7)
        self.assertEqual(sum(len(asset['profiles']) for asset in actual['assets'].values()), 14)
        self.assertEqual(actual['assets'], {name: value for name, value in original['assets'].items() if name != 'RedBarrel'})
        record = actual['qualification']['deferred_assets']['RedBarrel']
        self.assertTrue(record['original_scale_preserved'])
        self.assertFalse(record['asset_removed_from_population'])
        self.assertFalse(actual['qualification']['raw_files_modified'])
        self.assertTrue(actual['qualification']['native_qa_unchanged'])

    def test_both_profiles_and_independent_native_scene_ids_required(self):
        for cases in (self.cases[:1], [self.cases[0], self.cases[0]],
                      [self.cases[0], dict(id='second', profile_id='iso125')]):
            with self.subTest(cases=cases), self.assertRaises(scale.ScaleContractError):
                defer.derive(self.source, cases, self.evidence)

    def test_already_deferred_or_unexpected_retained_asset_count_rejected(self):
        source = copy.deepcopy(self.source)
        source['assets'].pop('RedBarrel')
        with self.assertRaises(scale.ScaleContractError):
            defer.derive(source, self.cases, self.evidence)
        source = copy.deepcopy(self.source)
        source['assets'].pop('WoodBox')
        with self.assertRaises(scale.ScaleContractError):
            defer.derive(source, self.cases, self.evidence)


if __name__ == '__main__':
    unittest.main()
