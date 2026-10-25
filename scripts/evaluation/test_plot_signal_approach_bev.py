import numpy as np

from plot_signal_approach_bev import camera_xy, horizontal_angle


def test_front_axle_camera_translation_rotates_with_heading():
    assert np.allclose(camera_xy([0., 0.], 0., 1.9, 0.), [-1.1, 0.])
    assert np.allclose(camera_xy([0., 0.], 90., 1.9, 0.), [0., -1.1])


def test_target_right_of_northbound_camera_is_negative_angle():
    assert horizontal_angle([0., 0.], 90., [1., 0.]) == -90.
    assert horizontal_angle([0., 0.], 0., [1., 0.]) == 0.
    assert horizontal_angle([0., 0.], 359., [1., 0.]) == 1.
