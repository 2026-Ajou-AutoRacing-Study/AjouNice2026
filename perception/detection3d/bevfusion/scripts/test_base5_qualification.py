import unittest

from prepare_base5_qualification_samples import choose
from summarize_base5_qualification import summarize_inference


class QualificationTest(unittest.TestCase):
    def records(self):
        return [dict(id=str(i), family='family'+str(i%6),
                     density_rank_nonzero_roi_returns=i,
                     nearest_gt_m=i/20, farthest_gt_m=i/4,
                     rear_gt=i%2==0, labels=[i%4]) for i in range(1000)]

    def test_fixed_distinct_groups_not_input_order(self):
        records = self.records()
        selected = choose(records)
        self.assertEqual(selected, choose(list(reversed(records))))
        self.assertEqual(len({r['id'] for r in selected}), 300)
        self.assertEqual(sum(r['sample_group']=='representative' for r in selected), 200)
        self.assertEqual(sum(r['sample_group']=='stress' for r in selected), 100)

    def test_small_population_rejected(self):
        with self.assertRaises(ValueError):
            choose(self.records()[:250])

    def test_separate_stress_statistics_and_duplicate_rejection(self):
        selected = choose(self.records())
        rows = [dict(id=r['id'], prepare_ms=1, model_ms=2,
                     service_ms=10 if r['sample_group']=='representative' else 100)
                for r in selected]
        case = dict(valid=True, shape='448x896', mode='separated', per_sample=rows,
                    steady_peak_allocated_mib=10, stages={})
        fixture = dict(source=dict(samples=selected), cases=[case])
        groups = summarize_inference(fixture)[0]['groups']
        self.assertEqual(groups['representative']['service_ms']['mean'], 10)
        self.assertEqual(groups['stress']['service_above_13hz_budget'], 100)
        rows[-1] = rows[0]
        with self.assertRaises(ValueError):
            summarize_inference(fixture)


if __name__ == '__main__':
    unittest.main()
