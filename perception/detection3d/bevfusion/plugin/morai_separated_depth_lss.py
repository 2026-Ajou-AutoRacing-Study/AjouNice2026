"""Explicit opt-in model type. E25 and old rectangular defaults stay unchanged."""
from mmdet3d.registry import MODELS
from .morai_rectangular_bev import MoraiRectangularDepthLSSTransform
from .morai_pool_optimization import enable_pool_optimization


@MODELS.register_module()
class MoraiSeparatedRectangularDepthLSSTransform(MoraiRectangularDepthLSSTransform):
    def __init__(self,*args,pool_cache_limit_bytes=256*2**20,pool_backward_mode='legacy',
                 pool_compact_indices=False,pool_geometry_mode='legacy',**kwargs):
        super().__init__(*args,**kwargs)
        enable_pool_optimization(self,'separated',cache_limit_bytes=pool_cache_limit_bytes,
                                 backward_mode=pool_backward_mode,compact_indices=pool_compact_indices,
                                 geometry_mode=pool_geometry_mode)
