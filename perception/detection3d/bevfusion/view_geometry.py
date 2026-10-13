"""Shared training/runtime config-owned projection geometry restoration."""
import torch


def restore_view_geometry(view):
    """Reset nonlearned bounds/frustum after loading a different-ROI checkpoint.

    dx/bx/nx have the same tensor shapes for every ROI, so a checkpoint loader
    cannot detect stale values by shape. Learned depth/image weights stay intact.
    """
    bounds = [view.xbound, view.ybound, view.zbound]
    dx = torch.tensor([b[2] for b in bounds])
    bx = torch.tensor([b[0] + b[2] / 2 for b in bounds])
    nx = torch.tensor([round((b[1] - b[0]) / b[2]) for b in bounds])
    frustum = view.create_frustum()
    if frustum.shape[0] != view.D:
        raise ValueError('depth bin count disagrees with configured depth head')
    with torch.no_grad():
        for name, value in [('dx', dx), ('bx', bx), ('nx', nx)]:
            target = getattr(view, name)
            target.copy_(value.to(device=target.device, dtype=target.dtype))
        view.frustum = torch.nn.Parameter(
            frustum.detach().to(device=view.dx.device), requires_grad=False)
