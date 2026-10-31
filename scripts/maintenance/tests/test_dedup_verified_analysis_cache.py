import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('dedup',Path(__file__).parents[1]/'dedup_verified_analysis_cache.py')
dedup=importlib.util.module_from_spec(spec);spec.loader.exec_module(dedup)


class DedupTests(unittest.TestCase):
    def fixture(self,root):
        old=root/'planner_logs/analysis/old/data';new=root/'planner_logs/analysis/new/data'
        old.mkdir(parents=True);new.mkdir(parents=True)
        path,peer=old/'image.bin',new/'image.bin'
        path.write_bytes(b'same');peer.write_bytes(b'same')
        plan=root/'plan.json'
        plan.write_text(json.dumps(dict(schema='verified_cache_dedup_plan_v1',
            source_roots=[str(old)],peer_roots=[str(new)],files=[dict(path=str(path),peer=str(peer),
                sha256=dedup.digest(path),bytes=4,source_stat=dedup.state(path),peer_stat=dedup.state(peer))])))
        return plan,path,peer

    def test_dry_run_then_atomic_dedup_preserves_bytes_and_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);plan,path,peer=self.fixture(root)
            receipt=dedup.run(plan,root/'dry.json',repository=root)
            self.assertTrue(receipt['passed']);self.assertNotEqual(path.stat().st_ino,peer.stat().st_ino)
            result=dedup.run(plan,root/'apply.json',True,root)
            self.assertEqual(result['completed'],1);self.assertEqual(path.stat().st_ino,peer.stat().st_ino)
            self.assertEqual(path.read_bytes(),b'same');self.assertEqual(peer.read_bytes(),b'same')

    def test_changed_file_and_non_cache_path_fail_before_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);plan,path,peer=self.fixture(root);inode=path.stat().st_ino
            peer.write_bytes(b'new!')
            with self.assertRaises(ValueError):dedup.run(plan,root/'changed.json',True,root)
            self.assertEqual(path.stat().st_ino,inode);self.assertEqual(path.read_bytes(),b'same')
            document=json.loads(plan.read_text());document['source_roots']=[str(root/'rosbags/data')]
            plan.write_text(json.dumps(document))
            with self.assertRaises(ValueError):dedup.run(plan,root/'outside.json',True,root)

    def test_symlink_is_not_replaced(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);plan,path,peer=self.fixture(root)
            path.unlink();path.symlink_to(peer)
            with self.assertRaises(ValueError):dedup.run(plan,root/'symlink.json',True,root)
            self.assertTrue(path.is_symlink())
