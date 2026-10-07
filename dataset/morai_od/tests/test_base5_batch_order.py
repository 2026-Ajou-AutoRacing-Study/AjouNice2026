import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from prepare_base5_batch_order import prepare
from base5_capture_safety import atomic_json,sha256
import generate_base5_independent as gen


class BatchOrderTest(unittest.TestCase):
    def test_retains_exact_payload_rows_and_full_motion_quota(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);rows=[]
            for i,motion in enumerate(('running','static','walking','standing')):
                sp=root/f'{i}.json';mp=root/f'{i}_meta.json'
                atomic_json(sp,dict(actor=i));atomic_json(mp,dict(runtime_actors=[dict(motion=motion)]))
                rows.append(dict(id=str(i),scenario=str(sp),metadata=str(mp),scenario_sha256=sha256(sp),split='train'))
            atomic_json(root/'manifest.json',dict(scenes=rows,config=dict(families={'a':4})))
            output=gen.geo.load_json(prepare(root/'manifest.json',root/'out'))
            self.assertEqual([r['id'] for r in output['scenes']],['1','3','0','2'])
            self.assertEqual({r['id']:r for r in output['scenes']},{r['id']:r for r in rows})
            self.assertEqual(output['collection_order']['stationary'],2)
            self.assertEqual(output['collection_order']['animated'],2)
            self.assertEqual(gen.geo.load_json(root/'manifest.json')['scenes'],rows)
            with self.assertRaises(FileExistsError):prepare(root/'manifest.json',root/'out')
