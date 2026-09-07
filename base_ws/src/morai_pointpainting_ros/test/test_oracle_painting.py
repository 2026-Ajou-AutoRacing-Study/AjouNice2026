import numpy as np

from morai_pointpainting_ros.oracle_painting import (
    override_semantics_in_oriented_boxes,
)


def test_oracle_override_changes_only_actor_points():
    points = np.zeros((4, 9), dtype=np.float32)
    points[:, 4] = 1.0
    points[:, :3] = np.asarray(
        [[0.0, 0.0, 0.0], [0.4, 0.2, 0.3], [1.0, 0.0, 0.0], [0.0, 0.0, -0.9]]
    )
    boxes = np.asarray([[0.0, 0.0, 0.0, 1.0, 0.8, 1.8, 0.0]])

    result, count = override_semantics_in_oriented_boxes(
        points,
        boxes,
        semantic_index=1,
        bottom_clearance_m=0.15,
        eligible_mask=[True, True, True, False],
    )

    assert count == 2
    np.testing.assert_array_equal(result[:2, 4:9], [[0, 1, 0, 0, 0]] * 2)
    np.testing.assert_array_equal(result[2:, 4:9], [[1, 0, 0, 0, 0]] * 2)


def test_oracle_swap_preserves_probability_values():
    points = np.zeros((1, 9), dtype=np.float32)
    points[0, :4] = [0.0, 0.0, 0.0, 1.0]
    points[0, 4:9] = [0.8, 0.1, 0.05, 0.05, 0.0]
    boxes = np.asarray([[0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0]])

    result, count = override_semantics_in_oriented_boxes(
        points, boxes, semantic_index=1, mode="swap_background"
    )

    assert count == 1
    np.testing.assert_allclose(result[0, 4:9], [0.1, 0.8, 0.05, 0.05, 0.0])
