"""Exact-copy/provenance tests; actual historical contracts are checked live offline."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_redbarrel_mixed_replay as prep


class RedMixedReplayTests(unittest.TestCase):
    def write(self,path,value):
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(value))
        return path

    def fixture(self,root):
        policy=dict(assets=dict(RedBarrel=dict(historical_test_fixture=True)))
        pp=self.write(root/'policy'/'old.json',policy)
        historical=root/'historical'
        entries=[]
        for ordinal,ident in enumerate(prep.IDS):
            meta=dict(spec=dict(seed=ordinal),runtime_actors=[dict(name='RedBarrel',unique_id=8500),
                dict(name='RedBarrel',unique_id=8501)])
            scene=root/'preserved'/ident/'scene.json'
            scene.parent.mkdir(parents=True,exist_ok=True)
            scene.write_text('{"objectList": []}  \n')
            metadata=self.write(scene.parent/'metadata.json',meta)
            capture=historical/f'chunk_{ordinal:04d}'/'capture'/'captures'/ident
            capture.mkdir(parents=True)
            (capture/'scenario.json').write_bytes(scene.read_bytes())
            self.write(capture/'metadata.json',meta)
            self.write(capture/'audit.json',dict(review_reasons=['historical_size_failure']))
            entries.append(dict(id=ident,scenario=str(scene),metadata=str(metadata),scenario_sha256=prep.sha256(scene)))
        manifest=self.write(root/'registered'/'manifest.json',dict(
            config=dict(capture=dict(),asset_scale_policy=policy),scenes=entries,count=2))
        return manifest,pp,historical

    def test_six_replays_keep_noncanonical_payload_bytes_and_preserved_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            manifest,policy,historical=self.fixture(root)
            before={path:path.read_bytes() for path in root.rglob('*') if path.is_file()}
            # Only fixture validation is mocked. The real historical deck was
            # independently exercised against unmodified validate_entry.
            with patch.object(prep.scale,'freeze_policy',return_value={}), \
                    patch.object(prep.scale,'verify_frozen_policy'), \
                    patch.object(prep,'validate_entry') as validate:
                path=prep.prepare(manifest,policy,historical,root/'replay')
            value=json.loads(path.read_text())
            self.assertEqual(value['count'],6)
            self.assertEqual(validate.call_count,8)
            self.assertEqual(len({entry['scenario_sha256'] for entry in value['scenes']}),1)
            self.assertFalse(value['redbarrel_mixed_replay_registration']['asset_scale_probe_enabled'])
            self.assertFalse(value['redbarrel_mixed_replay_registration']['production_profile_reapproved'])
            self.assertEqual(before,{path:path.read_bytes() for path in before})
            for entry in value['scenes']:
                self.assertEqual(Path(entry['scenario']).read_bytes(),b'{"objectList": []}  \n')

    def test_single_parent_probe_flag_cannot_enable_a_mixed_bypass(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            manifest,policy,historical=self.fixture(root)
            value=json.loads(manifest.read_text())
            value['config']['capture']['asset_scale_probe']=True
            manifest.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError,'no probe-contract bypass'):
                prep.prepare(manifest,policy,historical,root/'unwritten')
            self.assertFalse((root/'unwritten').exists())

    def test_unbounded_repeats_fail_before_source_access(self):
        for repeats in (0,4,True):
            with self.subTest(repeats=repeats),self.assertRaises(ValueError):
                prep.prepare('/missing','/policy','/history','/out',repeats=repeats)


if __name__=='__main__':
    unittest.main()
