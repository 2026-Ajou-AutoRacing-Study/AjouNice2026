import os
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from report_base5_prebulk_cost import unique_storage


class StorageTests(unittest.TestCase):
    def test_qa_hardlink_not_counted_as_new_capture(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);p=root/'raw';p.write_bytes(b'x'*4096)
            os.link(p,root/'qa_link')
            self.assertEqual(unique_storage(root)['logical_bytes'],4096)
            (root/'copied').write_bytes(p.read_bytes())
            self.assertEqual(unique_storage(root)['logical_bytes'],8192)


if __name__=='__main__':unittest.main()
