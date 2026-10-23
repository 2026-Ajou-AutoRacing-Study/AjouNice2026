import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from base5_capture_safety import atomic_json, sha256
import prepare_base5_primary_range_qualification as qualification


class PrimaryQualificationTests(unittest.TestCase):
    def fixture(self, root):
        entries, metadata, plans = {}, {}, []
        ids = ['s%02d' % i for i in range(64)] + ['m%d' % i for i in range(8)]
        for index, ident in enumerate(ids):
            scene, meta_path = root / (ident + '.json'), root / (ident + '_metadata.json')
            atomic_json(scene, {'id': ident})
            spec = dict(seed=index)
            if index < 4:
                spec[qualification.primary.PLAN_KEY] = dict(logical_slot_id=ident,
                    cap_m=20 if index % 2 == 0 else 40, primary_data_id=index, primary_name='asset' + str(index))
                if index >= 2:
                    spec[qualification.front.PLAN_KEY] = dict(selected=True)
                plans.append(ident)
            meta = dict(spec=spec, runtime_actors=[dict(data_id=index, motion='static' if index < 64 else 'walking')])
            atomic_json(meta_path, meta)
            metadata[ident] = meta
            entries[ident] = dict(id=ident, scenario=str(scene), scenario_sha256=sha256(scene), metadata=str(meta_path))
        previous = dict(source_manifest_sha256='previous', static=[dict(id=ident, sha256=entries[ident]['scenario_sha256']) for ident in ids[:64]],
                        animated=[dict(id=ident, sha256=entries[ident]['scenario_sha256']) for ident in ids[64:]],
                        policy='front-compatible-policy')
        deck = dict(count=5000, scenes=list(entries.values()), config={qualification.primary.CONFIG_KEY: dict(source_manifest_sha256='previous'),
                    'frozen_specs_sha256': 'frozen'})
        return deck, previous, entries, metadata, plans

    def test_balanced_four_strata_are_order_independent(self):
        with tempfile.TemporaryDirectory() as temporary:
            deck, previous, entries, metadata, plans = self.fixture(Path(temporary))
            a = qualification.select_strata(entries, metadata, plans)
            b = qualification.select_strata(entries, metadata, list(reversed(plans)))
            self.assertEqual(a, b)
            self.assertEqual({(r['has_front_plan'], r['cap_band']) for r in a},
                             {(has_front, band) for has_front in (False, True) for band in ('20', '30plus')})
            self.assertEqual(len({r['source_id'] for r in a}), 4)
            for row in a:
                self.assertEqual(row['range_plan'], metadata[row['source_id']]['spec'][qualification.primary.PLAN_KEY])

    def test_missing_stratum_is_a_contract_error(self):
        with tempfile.TemporaryDirectory() as temporary:
            _, _, entries, metadata, plans = self.fixture(Path(temporary))
            with self.assertRaisesRegex(qualification.primary.RangeContractError, 'front absent/present'):
                qualification.select_strata(entries, metadata, plans[:-1])

    def test_prepare_preserves_old_regression_ids_and_registers_every_range_contract(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            deck, previous, entries, metadata, plans = self.fixture(root)
            manifest, old = root / 'manifest.json', root / 'previous.json'
            atomic_json(manifest, deck)
            atomic_json(old, previous)
            pins = {str(p): sha256(p) for p in (manifest, old)}
            with patch.object(qualification, 'validate_source', return_value=(entries, metadata, plans)):
                output = qualification.prepare(manifest, old, root / 'registered')
            doc = qualification.read(output / 'registration.json')
            compatible = qualification.read(output / 'front_registration.json')
            self.assertEqual(doc['schema'], qualification.SCHEMA)
            self.assertEqual(doc['range_static_count'], 4)
            self.assertEqual({r['id'] for r in doc['range_static']}, set(plans))
            self.assertEqual([r['id'] for r in compatible['static']], [r['id'] for r in previous['static']])
            self.assertEqual([r['id'] for r in compatible['animated']], [r['id'] for r in previous['animated']])
            self.assertFalse(doc['actual_deficit_fabricated'])
            self.assertFalse(doc['live_qualified'])
            for name, count in (('static64', 64), ('animated8', 8), ('range_static', 4), ('range_recovery4', 4)):
                subset = qualification.read(output / name / 'manifest.json')
                self.assertEqual(subset['count'], count)
                self.assertEqual(sha256(output / name / 'manifest.json'), doc['manifests'][name]['sha256'])
            self.assertTrue(all(sha256(path) == digest for path, digest in pins.items()))

    def test_untargeted_regression_payload_cannot_change(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            deck, previous, entries, metadata, plans = self.fixture(root)
            previous['static'][5]['sha256'] = 'other'
            manifest, old = root / 'manifest.json', root / 'previous.json'
            atomic_json(manifest, deck)
            atomic_json(old, previous)
            with patch.object(qualification, 'validate_source', return_value=(entries, metadata, plans)):
                with self.assertRaisesRegex(qualification.primary.RangeContractError, 'Untargeted regression'):
                    qualification.prepare(manifest, old, root / 'registered')

    def test_existing_output_is_never_replaced(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(FileExistsError):
                qualification.prepare(Path('/source'), Path('/registration'), Path(temporary))


if __name__ == '__main__':
    unittest.main()
