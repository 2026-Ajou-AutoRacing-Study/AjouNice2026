import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from repair_base5_portable_mirror import mirror


class PortableMirrorTest(unittest.TestCase):
    def fixture(self, root):
        source, repair = root / 'source', root / 'repair'
        source.mkdir(); repair.mkdir()
        files = {}
        for name, payload in (('data/points.bin', b'original'), ('infos/512x1056/calibration.json', b'{}')):
            directory = repair if name.startswith('infos/') else source
            path = directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
            files[name] = dict(bytes=len(payload), sha256=hashlib.sha256(payload).hexdigest())
        (source / 'manifest.json').write_text(json.dumps(dict(files=files, bytes=10, box_geometry_policy='native_rpy_fixed_yaw_enclosure_v1')))
        return source, repair

    def test_missing_metadata_restored_without_sensor_copy(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); source, repair = self.fixture(root)
            result = mirror(source, repair, root / 'out')
            self.assertTrue(result['valid'])
            self.assertEqual(result['repaired'], ['infos/512x1056/calibration.json'])
            self.assertEqual((root / 'out/data/points.bin').stat().st_ino, (source / 'data/points.bin').stat().st_ino)
            self.assertFalse((source / 'infos').exists())
            self.assertEqual((source / 'manifest.json').read_bytes(), (root / 'out/manifest.json').read_bytes())
            with self.assertRaises(FileExistsError):
                mirror(source, repair, root / 'out')

    def test_wrong_metadata_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); source, repair = self.fixture(root)
            (repair / 'infos/512x1056/calibration.json').write_bytes(b'[]')
            with self.assertRaisesRegex(ValueError, 'does not match'):
                mirror(source, repair, root / 'out')

    def test_copy_mode_preserves_bytes_without_shared_inode(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); source, repair = self.fixture(root)
            result = mirror(source, repair, root / 'out', 'copy')
            self.assertEqual(result['copy_mode'], 'copy')
            self.assertNotEqual((root / 'out/data/points.bin').stat().st_ino, (source / 'data/points.bin').stat().st_ino)
            self.assertEqual((root / 'out/data/points.bin').read_bytes(), b'original')


if __name__ == '__main__':
    unittest.main()
