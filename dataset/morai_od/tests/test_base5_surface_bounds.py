import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from report_base5_surface_bounds import bounds, compare_bounds, local_surface, project


def test_cylinder_has_actual_top_bottom_and_no_box_corner_assumption():
    surface = local_surface([2.,4.,6.], cylinder=True)
    assert surface.shape == (720,3)
    np.testing.assert_allclose(surface.min(0), [-1.,-2.,-3.])
    np.testing.assert_allclose(surface.max(0), [1.,2.,3.])
    np.testing.assert_allclose((surface[:,0]**2 + (surface[:,1]/2)**2), 1.)
    assert not ((surface[:,0] == 1.) & (surface[:,1] == 2.)).any()


def test_oversize_box_detectable_when_containment_still_passes():
    result = compare_bounds([10.,10.,90.,90.], [0.,0.,100.,100.])
    assert result['observed_to_projected_extent_ratio'] == [.8,.8]
    assert result['mean_absolute_edge_residual_px'] == 10.
    assert compare_bounds([10.,10.,90.,90.],[10.,10.,90.,90.])['mean_absolute_edge_residual_px'] == 0.


def test_projection_uses_optical_transform_and_depth_gate():
    camera = dict(lidar_to_optical=np.eye(4).tolist(), intrinsic=[[100.,0.,50.],[0.,100.,40.],[0.,0.,1.]])
    uv, valid = project(np.asarray([[1.,2.,10.],[0.,0.,-1.]]), camera)
    np.testing.assert_allclose(uv[0], [60.,60.])
    assert valid.tolist() == [True,False]
    assert np.isnan(uv[1]).all()


@pytest.mark.parametrize('size', ([0,1,1],[1,-1,1],[1,float('nan'),1],[1,2]))
def test_invalid_geometry_rejected(size):
    with pytest.raises(ValueError):
        local_surface(size)


def test_bounds_are_xyxy_not_row_col():
    assert bounds([[5.,8.],[1.,9.],[4.,2.]]) == [1.,2.,5.,9.]


@pytest.mark.parametrize('a,b', [([0,0,0,1],[0,0,1,1]), ([0,0,1,1],[0,0,float('inf'),1])])
def test_invalid_extent_rejected(a,b):
    with pytest.raises(ValueError):
        compare_bounds(a,b)
