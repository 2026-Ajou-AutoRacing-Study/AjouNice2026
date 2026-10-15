import copy
import json
import math
import pathlib
import random
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from profile_pipeline_cpu_rotated_nms import baseline, candidate_indices, candidate_nms, reference_indices, operating_rows


def box(x=0., y=0., score=.5, cls="obstacle", size=(2., 1., 1.), yaw=0.):
    return dict(center=[x, y, 0.], size=list(size), yaw=yaw, score=score, class_name=cls)


def outcome(function, detections, threshold):
    try:
        return "ok", function(detections, threshold)
    except Exception as error:
        return "error", type(error), str(error)


class LazyNmsTests(unittest.TestCase):
    def exact(self, detections, threshold=.5):
        before = json.dumps(detections, sort_keys=True)
        reference = outcome(reference_indices, detections, threshold)
        candidate = outcome(candidate_indices, detections, threshold)
        self.assertEqual(reference, candidate)
        self.assertEqual(json.dumps(detections, sort_keys=True), before)
        if reference[0] == "ok":
            left = baseline.class_aware_rotated_bev_nms(detections, threshold)
            right = candidate_nms(detections, threshold)
            self.assertEqual(len(left), len(right))
            self.assertTrue(all(a is b for a, b in zip(left, right)))

    def test_ties_original_order_class_normalization_and_strict_boundary(self):
        detections = [box(score=.5, cls=" OBSTACLE "), box(score=.5), box(10., score=.2, cls="vehicle")]
        self.exact(detections)
        self.assertEqual(candidate_indices(detections, .5), [0, 2])
        self.exact(detections, 1.)
        self.assertEqual(candidate_indices(detections, 1.), [0, 1, 2])
        for threshold in (0., .25, .5, .75, 1.):
            self.exact([box(), box(2.), box(2. + 1.e-12)], threshold)

    def test_missing_and_malformed_unused_geometry_remains_lazy(self):
        inputs = [{"score": .5}, {"score": .5, "size": []},
                  {"score": .5, "size": [0., 1.]}, {"score": .5, "size": None},
                  {"score": .5, "size": [1.]}, {"score": .5, "size": [1., 1.]},
                  {"score": .5, "size": [1., 1.], "center": ["bad", 0.]},
                  box(yaw=float("inf")), box(size=(-2., -1., 1.)), box(x=float("nan")),
                  box(size=(float("nan"), 1., 1.)), box(x=float("inf"))]
        for detection in inputs:
            for partner in (box(), box(100.), box(cls="vehicle"), box(size=(0., 1., 1.))):
                for order in ([detection, partner], [partner, detection]):
                    with self.subTest(order=order):
                        self.exact(order)
            self.exact([detection])

    def test_invalid_scores_thresholds_and_nonfinite_matching(self):
        for threshold in (-1., 2., float("nan"), float("inf"), "bad"):
            self.exact([box()], threshold)
        for score in (float("nan"), float("inf"), -float("inf"), "bad", None):
            self.exact([box(score=score), box(score=.5)])
        for value in (None, False, 0, "", " Vehicle ", "unknown"):
            self.exact([box(cls=value), box(cls=value), box(10., cls="obstacle")])

    def test_cache_is_per_invocation_and_returned_mutable_aliases_match(self):
        shared = box()
        detections = [shared, shared, box(5.)]
        self.exact(detections, 1.)
        output = candidate_nms(detections, 1.)
        self.assertIs(output[0], output[1])
        output[0]["center"][0] = 100.
        self.assertEqual(shared["center"][0], 100.)
        # Subsequent invocation must read the edited input, never previous cache.
        self.exact(detections, .5)
        detections[1] = box(5., score=.9)
        self.exact(detections, .5)

    def test_seeded_random_geometry_matches_retained_indices(self):
        rng = random.Random(261014)
        for _ in range(400):
            detections = [box(rng.uniform(-12., 12.), rng.uniform(-8., 8.),
                score=rng.choice((.1, .5, .9)), cls=rng.choice(("vehicle", "pedestrian", "obstacle")),
                size=(rng.uniform(.05, 6.), rng.uniform(.05, 3.), 1.), yaw=rng.uniform(-math.pi, math.pi))
                for _ in range(rng.randrange(0, 40))]
            self.exact(detections, rng.choice((0., .1, .5, .8, 1.)))

    def test_actual_bridge_class_filter_precedes_nms_but_follows_server_top100(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = pathlib.Path(temporary) / "predictions.json"
            detections = [box(score=.2, cls="vehicle"), box(score=.5, cls="vehicle"),
                          box(score=.08, cls="pedestrian"), box(score=.1, cls="pedestrian"),
                          box(score=.12, cls="obstacle")]
            path.write_text(json.dumps(dict(rows=[dict(predictions=detections)])))
            self.assertEqual(len(operating_rows(path, "server-envelope")[0]), 4)
            actual = operating_rows(path, "bridge")[0]
            self.assertEqual([point["class_name"] for point in actual], ["vehicle", "obstacle", "pedestrian"])
            detections = [box(score=.2, cls="vehicle") for _ in range(100)] + [box(score=.19)]
            path.write_text(json.dumps(dict(rows=[dict(predictions=detections)])))
            self.assertEqual(operating_rows(path, "bridge")[0], [])


if __name__ == "__main__":
    unittest.main()
