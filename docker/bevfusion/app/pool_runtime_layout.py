"""Reversible eval-only layout fusion after the unchanged separated CUDA pool.

The original [B,Z,X,Y,C] -> [B,C,Z,X,Y] -> Z-concat -> YX-transpose
materializes three layouts. This computes the same Z-major/C channel order in
one contiguous copy. Geometry, indices, arithmetic, dtype and kernel stay put.
"""


def fused_runtime_layout(output):
    if output.ndim != 5:
        raise ValueError('Expected separated pool output [B,Z,X,Y,C]')
    batch, z_cells, x_cells, y_cells, channels = output.shape
    return output.permute(0, 1, 4, 3, 2).contiguous().reshape(
        batch, z_cells * channels, y_cells, x_cells)


def _separated_pool(depth, feat, plan, backward_mode):
    from perception.detection3d.bevfusion.plugin.morai_pool_optimization import SeparatedPool
    return SeparatedPool.apply(depth, feat, plan, backward_mode)


class _FusedPoolLayout:
    def __init__(self, view):
        self.view = view
        self.original = view.bev_pool
        self.enabled = True
        self.report = dict(enabled=True, mode='separated_one_copy_rectangular',
                           new_cuda_kernel=False, dtype_or_geometry_changed=False,
                           scope='eval-only; unchanged separated kernel')
        view.bev_pool = self.pool
        view._runtime_fused_pool_layout = self

    def pool(self, geom, features):
        import torch
        if not self.enabled or self.view.training or torch.is_grad_enabled():
            raise RuntimeError('Fused pool layout requires enabled eval inference')
        state = self.view._pool_optimization
        if state['mode'] != 'separated':
            raise RuntimeError('Separated pool policy changed after installation')
        if not isinstance(features, (tuple, list)) or len(features) != 2:
            raise ValueError('Expected unchanged separated depth/feature pair')
        depth, feat = features
        plan = state['plan']
        if plan is None:
            raise RuntimeError('Geometry must be computed before pooling')
        if (tuple(depth.shape) != tuple(plan['input_shape']) or
                tuple(feat.shape[:-1]) != (*plan['input_shape'][:2], *plan['input_shape'][3:])):
            raise ValueError('Depth/feature dimensions differ from geometry plan')
        return fused_runtime_layout(_separated_pool(
            depth, feat, plan, state['backward_mode']))

    def close(self):
        if not self.enabled:
            return
        if self.view.bev_pool != self.pool or self.view._runtime_fused_pool_layout is not self:
            raise RuntimeError('Another owner changed the pool wrapper; refusing to overwrite it')
        self.view.bev_pool = self.original
        del self.view._runtime_fused_pool_layout
        self.enabled = False
        self.report['enabled'] = False

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def install_fused_pool_layout(view):
    """Install on the existing rectangular separated eval path, before warmup.

    No checkpoint parameters/buffers or original plugin source are modified.
    Square/legacy, training, and materialized-cache paths are not qualified.
    """
    if view.training:
        raise ValueError('Fused pool layout requires model.eval()')
    if 'MoraiRectangularDepthLSSTransform' not in {base.__name__ for base in type(view).__mro__}:
        raise ValueError('Fused pool layout requires the Base5 rectangular view transform')
    state = getattr(view, '_pool_optimization', None)
    if not isinstance(state, dict) or state.get('mode') != 'separated':
        raise ValueError('Fused pool layout requires existing separated pooling')
    if hasattr(view, '_runtime_fused_pool_layout'):
        raise ValueError('Fused pool layout is already installed')
    return _FusedPoolLayout(view)
