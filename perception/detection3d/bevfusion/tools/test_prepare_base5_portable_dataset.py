import importlib.util
import json
from pathlib import Path
import pickle
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('portable', Path(__file__).with_name('prepare_base5_portable_dataset.py'))
portable = importlib.util.module_from_spec(spec); spec.loader.exec_module(portable)


class PortableTest(unittest.TestCase):
    def fixture(self, root):
        capture = root / 'original' / 's1'; capture.mkdir(parents=True)
        (capture / 'points.bin').write_bytes(b'points')
        (capture / 'native_annotation.json').write_text('{"preserve":true}')
        images = {}
        for i in range(4):
            path = capture / ('c%d.png' % i); path.write_bytes(b'pixels')
            images[str(i)] = dict(img_path=str(path))
        row = dict(id='s1', capture_root=str(capture))
        ledger = root / 'ledger.jsonl'; ledger.write_text(json.dumps(row) + '\n')
        folder = root / 'infos'; folder.mkdir()
        record = dict(token='s1', scene_group_id='g1', lidar_points=dict(lidar_path=str(capture/'points.bin')), images=images)
        for split in ('train', 'val', 'test'):
            info = dict(metainfo=dict(classes=('vehicle','pedestrian','obstacle','animal')),
                        data_list=[record] if split == 'train' else [])
            (folder/('morai_base5_infos_'+split+'.pkl')).write_bytes(pickle.dumps(info))
        (folder/'calibration.json').write_text('{}')
        return ledger, folder

    def test_relocatable_and_annotation_preserved(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name); ledger, infos = self.fixture(root)
            out = root/'export'; result = portable.build(ledger, {'448':infos}, out, 1)
            self.assertEqual(result['scenes'], 1)
            moved = root/'moved'; out.rename(moved)
            portable.validate(moved, True)
            item = pickle.loads((moved/'infos/448/morai_base5_infos_train.pkl').read_bytes())['data_list'][0]
            self.assertFalse(Path(item['lidar_points']['lidar_path']).is_absolute())
            self.assertTrue((moved/'data/s1/native_annotation.json').exists())
            with self.assertRaises(FileExistsError): portable.build(ledger, {'448':infos}, moved, 1)

    def test_tamper_detected(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name); ledger, infos = self.fixture(root)
            out = root/'export'; portable.build(ledger, {'448':infos}, out, 1)
            (out/'data/s1/points.bin').write_bytes(b'broken')
            with self.assertRaises(ValueError): portable.validate(out, True)

    def test_escape_rejected(self):
        for path in ('/etc/passwd', '../outside', 'data/../../outside'):
            with self.assertRaises(ValueError): portable.safe_relative(path)


if __name__ == '__main__': unittest.main()
