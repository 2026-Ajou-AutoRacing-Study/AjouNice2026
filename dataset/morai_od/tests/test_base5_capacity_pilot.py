import json
from pathlib import Path
import sys
import tempfile
import unittest
from collections import Counter

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from prepare_base5_capacity_pilot import COUNTS, prepare
from report_base5_capacity_pilot import report


class CapacityPilotTest(unittest.TestCase):
    def test_family_counts_and_source_rows_preserved(self):
        rows = []
        for family, count in COUNTS.items():
            for index in range(count + 3):
                rows.append(dict(id=f'{family}_{index}', family=family, subfamily=None,
                    distance_bounds=[6, 10], role='front', motion='static', split='train'))
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / 'source.json'
            source.write_text(json.dumps(dict(scenes=rows, count=len(rows))))
            result = json.loads(prepare(source, root / 'pilot').read_text())
            self.assertEqual(result['count'], 32)
            self.assertEqual(Counter(r['family'] for r in result['scenes']), COUNTS)
            indices = result['capacity_sample']['source_indices']
            self.assertEqual(len(set(indices)), 32)
            self.assertEqual(result['scenes'], [rows[i] for i in indices])
            with self.assertRaises(FileExistsError):
                prepare(source, root / 'pilot')

    def test_cost_includes_rejected_attempts_and_requires_safe_shutdown(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            capture = root / 'capture'
            cell = capture / 'cell' / 'round_0'
            cell.mkdir(parents=True)
            (capture / 'status.json').write_text(json.dumps(dict(stage='complete', elapsed_sec=30)))
            (capture / 'quota.json').write_text(json.dumps(dict(target=1, accepted=1)))
            manifest = root / 'manifest.json'
            manifest.write_text(json.dumps(dict(scenes=[dict(id='cell', family='highway')],
                config=dict(families=dict(highway=10000)))))
            attempts = []
            for kind, seconds in [('initial', 10), ('input_z_retry', 5)]:
                child = cell / kind
                child.mkdir()
                (child / 'status.json').write_text(json.dumps(dict(elapsed_sec=seconds)))
                attempts.append(dict(kind=kind, capture_root=str(child),
                    reasons=['ground_review'] if kind=='initial' else [],
                    shutdown=dict(park_verified=True)))
            payload = dict(selected=True, attempts=attempts)
            path = cell / 'report.json'
            path.write_text(json.dumps(payload))
            result = report(capture, manifest, root / 'cost.json')
            self.assertEqual(result['capture_attempts'], 2)
            self.assertEqual(result['review_reasons'], dict(ground_review=1))
            self.assertAlmostEqual(result['extrapolated_10k_hours_family_weighted'], 30*10000/3600)
            self.assertTrue(result['bulk_start_eligible'])
            payload['attempts'][0]['shutdown']['park_verified'] = False
            path.write_text(json.dumps(payload))
            self.assertFalse(report(capture, manifest, root / 'cost.json')['bulk_start_eligible'])
