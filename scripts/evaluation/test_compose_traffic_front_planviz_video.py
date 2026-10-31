import numpy as np
import pytest

import compose_traffic_front_planviz_video as compose


def test_offset_never_displays_unavailable_bev_frames():
    assert compose.screen_index(14.8, 14.85, 10., 1349) is None
    assert compose.screen_index(15., 14.85, 10., 1349) == 2
    assert compose.screen_index(160., 14.85, 10., 1349) is None


def test_viewport_must_exclude_outside_screen_coordinates():
    assert compose.validate_crop([461,401,519,511],1920,1080) == (461,401,519,511)
    for crop in ([-1,0,50,50], [0,0,0,50], [1900,0,100,100]):
        with pytest.raises(ValueError):
            compose.validate_crop(crop,1920,1080)


def test_composition_preserves_original_camera_and_footer_exactly():
    front = np.full((960,1280,3),75,dtype=np.uint8)
    bev = np.full((511,519,3),120,dtype=np.uint8)
    result = compose.compose(front,bev,88.2)
    assert result.shape == (960,1920,3)
    assert np.array_equal(result[:,:1280],front)
    assert np.all(result[920:,1280:] == 0)  # Bottom stays clear of player controls.
    missing = compose.compose(front,None,0.)
    assert np.array_equal(missing[:,:1280],front)
