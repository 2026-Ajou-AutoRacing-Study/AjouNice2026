import copy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from diagnose_base5_native_extent_repeats import camera_masks, mask_comparison, physical_payload, rgb_comparison


def test_physical_payload_ignores_only_object_uid_not_geometry_or_ego_identity():
    a = dict(objectList=[dict(UNIQUEID=8500, scale=dict(x=1., y=1., z=1.), pos=dict(z=3.))],
        egoVehicle=dict(UNIQUEID=-1), sensor=dict(m_SensorUniqueID=4))
    b = copy.deepcopy(a)
    b['objectList'][0]['UNIQUEID'] = 60000
    assert physical_payload(a) == physical_payload(b)
    b['objectList'][0]['pos']['z'] = 3.01
    assert physical_payload(a) != physical_payload(b)
    b = copy.deepcopy(a)
    b['egoVehicle']['UNIQUEID'] = -2
    assert physical_payload(a) != physical_payload(b)
    b = copy.deepcopy(a)
    b['sensor']['m_SensorUniqueID'] = 5
    assert physical_payload(a) != physical_payload(b)


def test_uid_and_rgba_mapping_not_row_order_or_colour_integer_identity():
    pixels = np.zeros((10,10,4), dtype=np.uint8)
    pixels[2:5,3:7] = [0,0,41,255]
    rows = 'Obstacle 3 2 7 5 60000 (0, 0, 41, 255)\n'
    masks = camera_masks(pixels, rows, [60001,60000])
    assert masks[60000][1]['bounds_xyxy'] == [3,2,7,5]
    assert masks[60000][1]['mask_pixels'] == 12
    assert masks[60001][1]['mask_pixels'] == 0
    assert mask_comparison(masks[60000][0],masks[60000][0]) == dict(
        iou=1., xor_pixels=0, identical=True, area_ratio_b_over_a=1.)
    # Parent/child encoding is explicit, never confused with a row ordinal.
    child = camera_masks(pixels, rows.replace('60000 ', '-6000001 '), [60000])
    assert np.array_equal(child[60000][0], masks[60000][0])


def test_same_rgba_different_uid_cannot_silently_merge_parents():
    pixels = np.zeros((10,10,4), dtype=np.uint8)
    rows = 'Obstacle 3 2 7 5 60000 (0, 0, 41, 255)\nObstacle 3 2 7 5 60001 (0, 0, 41, 255)'
    with pytest.raises(ValueError, match='different parent/class'):
        camera_masks(pixels, rows, [60000,60001])
    with pytest.raises(ValueError, match='RGBA'):
        camera_masks(pixels, 'Obstacle 3 2 7 5 60000 (0, 0, 41, 300)', [60000])


def test_mask_iou_detects_shape_change_without_any_native_geometry_assumption():
    a = np.zeros((10,10), bool)
    a[2:5,3:7] = True
    b = a.copy()
    b[5,3:7] = True
    value = mask_comparison(a,b)
    assert value['iou'] == .75
    assert value['xor_pixels'] == 4
    assert value['area_ratio_b_over_a'] == pytest.approx(4/3)
    assert not value['identical']


def test_rgb_difference_uses_signed_subtraction_and_only_observed_mask():
    a = np.zeros((2,2,3), np.uint8)
    b = np.full((2,2,3), 255, np.uint8)
    mask = np.zeros((2,2), bool)
    mask[0,0] = True
    b[0,0] = 1
    value = rgb_comparison(a,b,mask)
    assert value['union_mask_pixels'] == 1
    assert value['mean_absolute_channel_difference_0_255'] == 1.
    assert value['p95_absolute_channel_difference_0_255'] == 1.
    assert value['identical_rgb_pixel_fraction'] == 0.
