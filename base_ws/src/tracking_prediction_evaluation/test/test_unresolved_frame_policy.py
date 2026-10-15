from dataclasses import replace
from tracking_prediction_evaluation.pipeline_evaluator import EgoPoint
from tracking_prediction_evaluation.trajectory import TrajectoryPoint, TrajectorySegment
from tracking_prediction_evaluation.unresolved_frame_policy import resolved_frame_mask


def test_unresolved_frame_is_not_treated_as_false_positive_region():
    p = TrajectoryPoint(1.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        'vehicle', 1, 'unknown')
    s = TrajectorySegment('missing', 'vehicle', 1, 'unknown', (p,))
    ego = (EgoPoint(1.0, 0.0, 0.0, 0.0),)
    mask, report = resolved_frame_mask((s,), (), ego, [1.0])
    assert mask == {1.0: False}
    assert report['excluded_frames'] == 1
    assert resolved_frame_mask((s,), (s,), ego, [1.0])[0] == {1.0: True}
    distant = replace(s, points=(replace(p, x=40.0),))
    assert resolved_frame_mask((distant,), (), ego, [1.0])[0] == {1.0: True}
