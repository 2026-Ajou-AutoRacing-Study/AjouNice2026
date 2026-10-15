import json
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from select_base5_ready_captures import select


class ReadySelectionTest(unittest.TestCase):
    def test_unsafe_exit_refused_even_if_no_rows(self):
        with tempfile.TemporaryDirectory() as d:
            r=Path(d);(r/'shutdown_safety.json').write_text(json.dumps(dict(park_verified=False)))
            ledger=r/'ledger.jsonl';ledger.write_text('')
            with self.assertRaises(ValueError):select([ledger],r/'selected.jsonl')
            self.assertFalse((r/'selected.jsonl').exists())

    def test_overwrite_refused(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'selected.jsonl';p.write_text('original')
            with self.assertRaises(FileExistsError):select([],p)
            self.assertEqual(p.read_text(),'original')

    def test_replacement_is_canonical_and_original_ledgers_unchanged(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);ledgers=[];originals=[]
            for name in ('a','replacement'):
                session=root/name;session.mkdir();cap=session/'capture';cap.mkdir()
                (session/'shutdown_safety.json').write_text(json.dumps(dict(park_verified=True)))
                (cap/'metadata.json').write_text(json.dumps(dict(runtime_actors=[])))
                (cap/'telemetry.json').write_text(json.dumps(dict(objects_before={},objects_after={})))
                (cap/'audit.json').write_text(json.dumps(dict(review_reasons=[],capture_safety=dict(collision=dict(no_collision_verified=True)))))
                row=dict(id=name,status='accepted_candidate',capture_root=str(cap))
                if name!='a':row['source_cell_id']='a'
                p=session/'ledger.jsonl';p.write_text(json.dumps(row)+'\n');ledgers.append(p);originals.append(p.read_bytes())
            output=root/'selected.jsonl';self.assertEqual(select(ledgers,output),1)
            self.assertEqual(json.loads(output.read_text())['id'],'replacement')
            self.assertEqual([p.read_bytes() for p in ledgers],originals)


if __name__=='__main__':unittest.main()
