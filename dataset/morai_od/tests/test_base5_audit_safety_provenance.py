import copy
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_audit import audit_capture


class AuditSafetyProvenanceTests(unittest.TestCase):
    def audit(self,safety):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'LIDAR_5').mkdir()
            np.array([[5.,2.,-1.,1.]],dtype=np.float32).tofile(root/'LIDAR_5/test_Intensity.bin')
            (root/'LIDAR_5/test_Instance.txt').write_text('')
            meta=dict(runtime_actors=[],spec=dict(family='background'))
            if safety is not None:meta['capture_safety']=safety
            return audit_capture(root,meta)

    def test_safety_is_preserved_without_raw_or_source_mutation(self):
        safety=dict(collision=dict(observed=True,no_collision_verified=True,non_ego_uids=[]),
            support=dict(stationary_support_observed=True),evidence_path='immutable-source-proof')
        expected=copy.deepcopy(safety);result=self.audit(safety)
        self.assertEqual(result['review_reasons'],[])
        self.assertEqual(result['capture_safety'],expected)
        result['capture_safety']['collision']['no_collision_verified']=False
        self.assertEqual(safety,expected)

    def test_absent_or_failed_proof_is_not_fabricated(self):
        self.assertNotIn('capture_safety',self.audit(None))
        safety=dict(collision=dict(observed=True,no_collision_verified=False,non_ego_uids=[7]),
            support=dict(stationary_support_observed=True))
        result=self.audit(safety)
        self.assertIn('physical_collision_during_capture_window',result['review_reasons'])
        self.assertFalse(result['capture_safety']['collision']['no_collision_verified'])


if __name__=='__main__':unittest.main()
