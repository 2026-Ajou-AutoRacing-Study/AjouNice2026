import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_base5_capture import acquire_live_master_lock


class LiveMasterLockTest(unittest.TestCase):
    def test_second_output_cannot_control_same_master(self):
        with tempfile.TemporaryDirectory() as directory:
            first=acquire_live_master_lock('http://localhost:11311',directory)
            try:
                with self.assertRaisesRegex(RuntimeError,'no MR action'):
                    acquire_live_master_lock('http://localhost:11311',directory)
                other=acquire_live_master_lock('http://localhost:11312',directory);other.close()
            finally:first.close()
            retry=acquire_live_master_lock('http://localhost:11311',directory);retry.close()


if __name__=='__main__':unittest.main()
