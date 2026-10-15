import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from base5_capture_safety import atomic_json
from verify_base5_partial_resume import snapshot, verify


class PartialResumeTests(unittest.TestCase):
    def fixture(self, root):
        ledger = root / 'selected_ledger.jsonl'
        ledger.write_text(json.dumps(dict(source_cell_id='a', status='accepted_candidate'))+'\n')
        raw = root / 'a_Intensity.bin'; raw.write_bytes(b'original')
        proof = root / 'before.json'; atomic_json(proof, snapshot(root))
        return ledger, raw, proof

    def test_append_preserves_old_selection_and_sensor_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); ledger, raw, proof = self.fixture(root)
            with ledger.open('a') as output:
                output.write(json.dumps(dict(source_cell_id='b', status='accepted_candidate'))+'\n')
            (root / 'b_Intensity.bin').write_bytes(b'new')
            result = verify(proof, root)
            self.assertTrue(result['passed']); self.assertEqual(result['preserved_selected'], 1)
            self.assertEqual(result['native_after'], 2)
            raw.write_bytes(b'changed')
            with self.assertRaises(ValueError): verify(proof, root)

    def test_replaced_selected_row_is_not_resume_preservation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); ledger, _, proof = self.fixture(root)
            ledger.write_text(json.dumps(dict(source_cell_id='a', status='different'))+'\n')
            with self.assertRaises(ValueError): verify(proof, root)

    def test_torn_or_duplicate_ledger_is_not_repaired_by_diagnostic(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); ledger, _, _ = self.fixture(root)
            raw = ledger.read_bytes()[:-1]; ledger.write_bytes(raw)
            with self.assertRaises(ValueError): snapshot(root)
            self.assertEqual(ledger.read_bytes(), raw)
            row = json.dumps(dict(source_cell_id='a', status='accepted_candidate'))+'\n'
            ledger.write_text(row*2)
            with self.assertRaises(ValueError): snapshot(root)


if __name__ == '__main__': unittest.main()
