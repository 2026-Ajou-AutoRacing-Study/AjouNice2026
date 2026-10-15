import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from share_base5_jpeg_portable import pack


class ArchiveTest(unittest.TestCase):
    def fixture(self, root):
        data=root/'portable'; data.mkdir()
        files={}
        for name,blob in (('camera.jpg',b'jpeg payload'),('mask.png',b'lossless labels')):
            (data/name).write_bytes(blob)
            files[name]=dict(bytes=len(blob),sha256=hashlib.sha256(blob).hexdigest())
        manifest=dict(profiles=[], scenes=1, split_counts=dict(train=1,val=0),files=files,
            bytes=sum(i['bytes'] for i in files.values()),
            image_policy=dict.fromkeys(('train','val','test'),'native_jpeg90'),
            auxiliary_contract='Preserve lossless labels')
        (data/'manifest.json').write_text(json.dumps(manifest))
        return data

    def test_archive_and_split_restore_preserve_every_file(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); data=self.fixture(root)
            result=pack(data,root/'backup','test.tar.zst')
            self.assertEqual(result['verified_archive_files'],3)
            part=root/'backup/parts'/result['parts'][0]['name']
            self.assertEqual(part.read_bytes(),(root/'backup/test.tar.zst').read_bytes())
            self.assertFalse((root/'backup/parts/COMPLETE.json').exists())
            self.assertEqual((data/'mask.png').read_bytes(),b'lossless labels')
            with self.assertRaises(FileExistsError): pack(data,root/'backup','test.tar.zst')

    def test_source_hash_and_input_policy_gate(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); data=self.fixture(root)
            (data/'mask.png').write_bytes(b'wrong')
            with self.assertRaises(ValueError): pack(data,root/'backup','test.tar.zst')
            self.assertFalse((root/'backup').exists())


if __name__=='__main__': unittest.main()
