import os
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from report_base5_prebulk_cost import unique_storage,observation_group,classified_storage


class StorageTests(unittest.TestCase):
    def test_classified_qa_storage_deduplicates_shared_hardlinks(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);a=root/'static';b=root/'moving';a.mkdir();b.mkdir()
            (a/'raw').write_bytes(b'x'*4096);os.link(a/'raw',b/'qa_link')
            (b/'other').write_bytes(b'x'*1024)
            sizes=classified_storage(dict(stationary=[a],shadow_animated=[b]))
            self.assertEqual(sizes['stationary'],4096);self.assertEqual(sizes['shadow_animated'],1024)

    def test_tunnel_setup_not_applied_to_all_animated_frames(self):
        a=dict(runtime_actors=[dict(motion='walking')],spec={})
        self.assertEqual(observation_group(a),'animated')
        self.assertEqual(observation_group(a,True),'intersection_animated')
        a['spec']['shadow_moving']=True
        self.assertEqual(observation_group(a,True),'shadow_animated')
        a['runtime_actors'][0]['motion']='standing'
        self.assertEqual(observation_group(a,True),'stationary')

    def test_qa_hardlink_not_counted_as_new_capture(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);p=root/'raw';p.write_bytes(b'x'*4096)
            os.link(p,root/'qa_link')
            self.assertEqual(unique_storage(root)['logical_bytes'],4096)
            (root/'copied').write_bytes(p.read_bytes())
            self.assertEqual(unique_storage(root)['logical_bytes'],8192)


if __name__=='__main__':unittest.main()
