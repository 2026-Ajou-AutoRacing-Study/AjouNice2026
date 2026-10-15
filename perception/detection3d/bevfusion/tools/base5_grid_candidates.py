"""Qualified spatial/depth COST candidates; never select a production model.

The three controlled grids share symmetric physical bounds, semantic ROI and
origin. Keep an additional unchanged-current control to expose padding cost.
LiDAR XY stride 8 and camera stride 2 require camera pitch = 4 * voxel pitch.
"""
from copy import deepcopy
from fractions import Fraction

SPATIAL_CASES = ('current075', 'common050', 'common075', 'common100')
PITCHES = dict(current075=.075, common050=.05, common075=.075, common100=.1)
SEMANTIC_ROI = [-100., -30., -3., 100., 30., 2.]


def geometry(case):
    if case not in SPATIAL_CASES:
        raise ValueError('Unknown qualified grid: ' + str(case))
    voxel = Fraction(str(PITCHES[case]))
    half_x = Fraction('100.8')
    half_y = Fraction('32.4' if case == 'current075' else '33.6')
    xy = [2 * half_x / voxel, 2 * half_y / voxel]
    if any(n.denominator != 1 or n.numerator % 16 for n in xy):
        raise ValueError('Sparse/SECOND/FPN grid must divide XY stride 16')
    nx, ny = [int(n) for n in xy]
    return dict(voxel=[float(voxel), float(voxel), .2], grid=[nx, ny, 41],
                origin=[-float(half_x), -float(half_y)],
                xbound=[-float(half_x), float(half_x), float(4*voxel)],
                ybound=[-float(half_y), float(half_y), float(4*voxel)],
                fusion_pitch_m=float(8*voxel), fusion_shape_yx=[ny//8, nx//8],
                raw_camera_shape_yx=[ny//4, nx//4])


def candidate_config(base, case, depth_step=.5, pool_cache_mib=None):
    if depth_step not in (.25, .5, 1.):
        raise ValueError('Qualified depth intervals are 0.25/0.5/1m')
    cfg = deepcopy(base)
    if list(cfg.point_cloud_range) != SEMANTIC_ROI:
        raise ValueError('Candidate must preserve the approved semantic ROI')
    if list(cfg.model.view_transform.image_size) != [512, 1056]:
        raise ValueError('Cost comparison requires the approved image shape')
    g = geometry(case)
    xmax, ymax = g['xbound'][1], g['ybound'][1]
    cfg.voxel_size = g['voxel']
    cfg.grid = g['grid']
    cfg.origin = g['origin']
    cfg.voxelization_range = [*g['origin'], -5., xmax, ymax, 3.]
    cfg.model.data_preprocessor.voxelize_cfg.update(
        voxel_size=g['voxel'], point_cloud_range=cfg.voxelization_range)
    cfg.model.pts_middle_encoder.sparse_shape = [g['grid'][1], g['grid'][0], 41]
    cfg.model.view_transform.update(xbound=g['xbound'], ybound=g['ybound'],
                                    dbound=[1., 100., depth_step])
    if pool_cache_mib is not None:
        if pool_cache_mib not in (256,512):raise ValueError('Qualified cache budgets are 256/512 MiB')
        cfg.model.view_transform.pool_cache_limit_bytes=pool_cache_mib*2**20
    head = cfg.model.bbox_head
    head.train_cfg.update(grid_size=g['grid'], voxel_size=g['voxel'],
                          point_cloud_range=[*g['origin'], -3., xmax, ymax, 2.])
    head.test_cfg.update(grid_size=g['grid'], voxel_size=g['voxel'][:2], pc_range=g['origin'])
    head.bbox_coder.update(voxel_size=g['voxel'][:2], pc_range=g['origin'],
                          post_center_range=SEMANTIC_ROI)
    cfg.model.img_backbone.init_cfg = None
    validate_geometry(cfg)
    return cfg


def validate_geometry(cfg):
    """Check nominal metric grids, not learned receptive-field center equality."""
    voxel = cfg.model.data_preprocessor.voxelize_cfg
    head = cfg.model.bbox_head
    assert head.train_cfg.out_size_factor == head.test_cfg.out_size_factor == head.bbox_coder.out_size_factor == 8
    assert cfg.model.view_transform.downsample == 2
    assert head.type == 'MoraiRectangularTransFusionHead'
    assert cfg.model.pts_middle_encoder.type == 'MoraiRectangularSparseEncoder'
    assert cfg.model.view_transform.type == 'MoraiSeparatedRectangularDepthLSSTransform'
    assert list(cfg.point_cloud_range) == list(head.bbox_coder.post_center_range) == SEMANTIC_ROI
    assert cfg.required_box_geometry_policy == 'native_rpy_fixed_yaw_enclosure_v1'
    for axis, bound in enumerate((cfg.model.view_transform.xbound, cfg.model.view_transform.ybound)):
        lo, hi, camera_pitch = map(lambda x: Fraction(str(x)), bound)
        lidar_pitch = Fraction(str(voxel.voxel_size[axis]))
        assert camera_pitch == 4*lidar_pitch
        assert lo == Fraction(str(voxel.point_cloud_range[axis]))
        assert hi == Fraction(str(voxel.point_cloud_range[axis+3]))
        n = (hi-lo)/lidar_pitch
        assert n.denominator == 1 and int(n) % 16 == 0
        assert int(n) == head.train_cfg.grid_size[axis] == head.test_cfg.grid_size[axis]
        assert Fraction(str(head.bbox_coder.pc_range[axis])) == lo
        assert Fraction(str(head.test_cfg.pc_range[axis])) == lo
        assert Fraction(str(head.train_cfg.point_cloud_range[axis])) == lo
        assert Fraction(str(head.train_cfg.voxel_size[axis])) == lidar_pitch
        assert Fraction(str(head.test_cfg.voxel_size[axis])) == lidar_pitch
        assert Fraction(str(head.bbox_coder.voxel_size[axis])) == lidar_pitch
    assert list(cfg.model.pts_middle_encoder.sparse_shape) == [head.train_cfg.grid_size[1], head.train_cfg.grid_size[0], 41]
    assert cfg.model.view_transform.dbound[0:2] == [1., 100.]
    assert (Fraction('99')/Fraction(str(cfg.model.view_transform.dbound[2]))).denominator == 1
