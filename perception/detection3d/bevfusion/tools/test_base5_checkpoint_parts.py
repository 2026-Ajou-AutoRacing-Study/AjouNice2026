import json
from pathlib import Path
import tempfile
import unittest

from base5_checkpoint_parts import split_checkpoint, verify_parts


class PartsTest(unittest.TestCase):
    def test_roundtrip_and_existing_output_preserved(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name); source=root/'epoch.pth'; source.write_bytes(bytes(range(256))*100)
            manifest=split_checkpoint(source,root/'parts',4096)
            output=root/'restored.pth'
            self.assertEqual(verify_parts(root/'parts',manifest,output),manifest['sha256'])
            self.assertEqual(source.read_bytes(),output.read_bytes())
            self.assertEqual(len(manifest['parts']),7)
            with self.assertRaises(FileExistsError): verify_parts(root/'parts',manifest,output)

    def test_corruption_has_no_complete_or_partial_output(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name); source=root/'epoch.pth'; source.write_bytes(b'abcd'*40)
            manifest=split_checkpoint(source,root/'parts',32)
            (root/'parts/part0001.bin').write_bytes(b'wrong')
            with self.assertRaises(ValueError): verify_parts(root/'parts',manifest,root/'restored')
            self.assertFalse((root/'restored').exists())
            self.assertFalse((root/'restored.partial').exists())

    def test_path_traversal_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name); source=root/'epoch.pth'; source.write_bytes(b'abcd')
            manifest=split_checkpoint(source,root/'parts',32)
            manifest['parts'][0]['name']='../epoch.pth'
            with self.assertRaises(ValueError): verify_parts(root/'parts',manifest)


if __name__ == '__main__': unittest.main()
