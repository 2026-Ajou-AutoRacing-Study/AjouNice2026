import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from correct_base5_collection_environment import correct
import generate_base5_independent as gen


class EnvironmentCorrectionTests(unittest.TestCase):
    def test_default_and_named_config_both_use_sunny13(self):
        default=json.loads(gen.CONFIG.read_text())
        named=json.loads((gen.CONFIG.parent/'base5_independent_5k_sunny13.json').read_text())
        self.assertEqual(default,named)
        self.assertEqual(default['collection_environment']['weather'],'sunny')
        self.assertEqual(default['collection_environment']['hour'],13)

    def test_correction_changes_header_not_source_geometry_or_split(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'old.json'
            deck=dict(dataset_id='base5_independent_5k_sunny11_261007',
                config=dict(dataset_id='base5_independent_5k_sunny11_261007',
                    collection_environment=dict(weather='sunny',hour=11)),
                scenes=[dict(id='one',scenario='immutable.json',metadata='immutable_meta.json',
                    scenario_sha256='immutable',split='train')],provenance={})
            source.write_text(json.dumps(deck));before=source.read_bytes()
            out=root/'new'/'manifest.json'
            result=correct(source,out,operator_confirmed=True)
            updated=json.loads(out.read_text())
            self.assertEqual(source.read_bytes(),before)
            self.assertEqual(updated['scenes'],deck['scenes'])
            self.assertEqual(updated['config']['collection_environment']['hour'],13)
            self.assertIn('sunny13',updated['dataset_id'])
            proof=json.loads(Path(result['record']).read_text())
            self.assertFalse(proof['independently_measured'])
            with self.assertRaises(FileExistsError):correct(source,out,operator_confirmed=True)
            with self.assertRaises(FileExistsError):correct(source,source,operator_confirmed=True)

    def test_no_weather_claim_without_operator_confirmation(self):
        with self.assertRaisesRegex(ValueError,'operator confirmation'):
            correct(Path('unused'),Path('unused'))


if __name__=='__main__':unittest.main()
