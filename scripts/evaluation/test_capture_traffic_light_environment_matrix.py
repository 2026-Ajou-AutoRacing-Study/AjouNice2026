import json
from pathlib import Path
import tempfile
import unittest

from capture_traffic_light_environment_matrix import check_environment, write_new, ENVIRONMENTS, select_environments


class MatrixTests(unittest.TestCase):
    def test_exact_readback_required(self):
        good={'status':'READBACK_MATCH','readback_match':True,'actual':{'weather':'Sunny','hour':13}}
        check_environment(good,'Sunny',13)
        for key,value in [('status','READ_ONLY_GET'),('readback_match',False)]:
            bad=dict(good);bad[key]=value
            with self.assertRaises(RuntimeError):check_environment(bad,'Sunny',13)
        with self.assertRaises(RuntimeError):check_environment(good,'Foggy',13)
        with self.assertRaises(RuntimeError):check_environment(good,'Sunny',11)

    def test_new_reports_never_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'report.json';write_new(p,{'verified':False})
            with self.assertRaises(FileExistsError):write_new(p,{'verified':True})
            self.assertEqual(json.loads(p.read_text()),{'verified':False})

    def test_exact_six_conditions(self):
        self.assertEqual(set(ENVIRONMENTS),{(w,h) for w in ['Sunny','Foggy'] for h in [11,13,15]})

    def test_subset_is_explicit_unique_known_and_does_not_change_default(self):
        self.assertEqual(select_environments('foggy11,foggy15'),[('Foggy',11),('Foggy',15)])
        self.assertEqual(select_environments(None), ENVIRONMENTS)
        for invalid in ['', 'sunny12', 'foggy11,foggy11', 'Foggy11']:
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):select_environments(invalid)


if __name__=='__main__':unittest.main()
