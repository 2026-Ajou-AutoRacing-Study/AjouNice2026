"""Initial failures must survive retries and cannot all become scale causes."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import report_base5_scale_stability as report


class ScaleStabilityReportTests(unittest.TestCase):
    def fixture(self,root):
        entries=[]
        for ident,condition in [('unit','original'),('large','scaled'),('missing','scaled')]:
            path=root/(ident+'_metadata.json')
            pair='q' if ident=='missing' else 'p'
            path.write_text(json.dumps(dict(scale_stability=dict(pair_id=pair,condition=condition,
                independent_geometry_unit=pair))))
            entries.append(dict(id=ident,metadata=str(path),scenario_sha256=ident+'_hash'))
        manifest=root/'manifest.json'
        manifest.write_text(json.dumps(dict(scenes=entries)))
        ledger=root/'raw.jsonl'
        rows=[dict(id='unit',scenario_sha256='unit_hash',status='accepted_candidate',duration_sec=7.),
              dict(id='large',scenario_sha256='large_hash',status='review',duration_sec=8.,
                   audit=dict(review_reasons=['scaled_native_bbox_size_mismatch'])),
              dict(id='large',scenario_sha256='large_hash',status='accepted_candidate',duration_sec=9.)]
        ledger.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        return manifest,ledger

    def test_failed_first_attempt_kept_and_unknown_not_declared_scale_caused(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            manifest,ledger=self.fixture(root)
            old=(manifest.read_bytes(),ledger.read_bytes())
            output=report.report(manifest,[ledger],root/'result.json')
            value=json.loads(output.read_text())
            self.assertEqual(value['initial_all_failure_rate']['failures'],1)
            self.assertEqual(value['observed_initial_scenes'],2)
            self.assertEqual(value['registered_scenes'],3)
            self.assertEqual(value['retry_attempts'],1)
            self.assertEqual(value['retry_timing']['sum_sec'],9.)
            self.assertEqual(value['confirmed_scale_failure_rate']['failures'],0)
            self.assertIsNone(value['confirmed_scale_failure_rate']['one_sided_95_upper_if_zero'])
            self.assertEqual(value['initial_failure_cause_counts'],dict(unknown=1))
            self.assertEqual(value['unobserved_registered_ids'],['missing'])
            self.assertEqual((manifest.read_bytes(),ledger.read_bytes()),old)

    def test_duplicate_ledgers_and_existing_output_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            manifest,ledger=self.fixture(root)
            with self.assertRaises(ValueError):
                report.report(manifest,[ledger,ledger],root/'result.json')
            output=root/'existing.json'
            output.write_text('{}')
            with self.assertRaises(FileExistsError):
                report.report(manifest,[ledger],output)

    def test_missing_times_are_not_zero_and_zero_failure_upper_is_exact(self):
        self.assertIsNone(report.timing([{}])['sum_sec'])
        self.assertEqual(report.timing([{}])['missing_durations'],1)
        self.assertAlmostEqual(report.rate(0,60)['one_sided_95_upper_if_zero'],.0487029133)
        self.assertIsNone(report.rate(1,60)['one_sided_95_upper_if_zero'])

    def test_matched_conditions_are_not_two_independent_geometry_trials(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            manifest,ledger=self.fixture(root)
            rows=[json.loads(line) for line in ledger.read_text().splitlines()][:2]
            for row in rows:
                row['status']='accepted_candidate'
                row['audit']=dict(review_reasons=[])
            ledger.write_text(''.join(json.dumps(row)+'\n' for row in rows))
            value=json.loads(report.report(manifest,[ledger],root/'result.json').read_text())
            self.assertEqual(value['initial_all_failure_rate']['distinct_source_geometry_units'],1)
            self.assertIsNone(value['initial_all_failure_rate']['one_sided_95_upper_if_zero'])
            self.assertAlmostEqual(value['initial_by_condition']['scaled']['one_sided_95_upper_if_zero'],.95)

    def test_identical_payload_repeats_cannot_create_an_independent_ci(self):
        rows=[dict(id='x',scenario_sha256='same',status='accepted_candidate'),
              dict(id='y',scenario_sha256='same',status='accepted_candidate')]
        self.assertEqual(len({row['scenario_sha256'] for row in rows}),1)
        # The reporter's end-to-end repeat suppression is exercised using a
        # registered repeated payload rather than treating two IDs as two n.
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            entries=[]
            for row in rows:
                meta=root/(row['id']+'.json')
                meta.write_text(json.dumps(dict(scale_probe=dict(condition='scaled'))))
                entries.append(dict(id=row['id'],scenario_sha256='same',metadata=str(meta)))
            manifest=root/'manifest.json'
            manifest.write_text(json.dumps(dict(scenes=entries)))
            ledger=root/'raw.jsonl'
            ledger.write_text(''.join(json.dumps(row)+'\n' for row in rows))
            value=json.loads(report.report(manifest,[ledger],root/'result.json').read_text())
            self.assertIsNone(value['initial_by_condition']['scaled']['one_sided_95_upper_if_zero'])
            self.assertIsNone(value['confirmed_scale_failure_rate']['one_sided_95_upper_if_zero'])


if __name__=='__main__':
    unittest.main()
