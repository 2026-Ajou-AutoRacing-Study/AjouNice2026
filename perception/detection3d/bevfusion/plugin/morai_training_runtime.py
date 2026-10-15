"""Opt-in Base5 training execution; unchanged model parameters/loss objective."""
from collections import OrderedDict

import torch
import torch.distributed as dist
from mmdet3d.registry import MODELS
from projects.BEVFusion.bevfusion.bevfusion import BEVFusion


@MODELS.register_module()
class MoraiTrainingRuntimeBEVFusion(BEVFusion):
    """Read scalar logs together rather than synchronize once for each loss.

    Loss and gradients retain their original reductions and order. Distributed
    runs use the parent's exact implementation; this candidate targets one GPU.
    No buffers or parameter names change, so audited initialization is reusable.
    """

    def __init__(self,*args,compile_camera=False,**kwargs):
        super().__init__(*args,**kwargs)
        self._base5_compile_camera=bool(compile_camera)
        self._base5_camera_compiled=False

    def extract_img_feat(self,*args,**kwargs):
        if self._base5_compile_camera and not self._base5_camera_compiled:
            # Lazy wrapping happens AFTER audited initialization/resume. A
            # bound callable does not introduce _orig_mod state_dict prefixes.
            self.img_backbone.forward=torch.compile(self.img_backbone.forward,dynamic=False)
            self._base5_camera_compiled=True
        return super().extract_img_feat(*args,**kwargs)

    def parse_losses(self, losses):
        if dist.is_available() and dist.is_initialized():
            return super().parse_losses(losses)
        values = []
        for name, value in losses.items():
            if torch.is_tensor(value):
                values.append((name, value.mean()))
            elif isinstance(value, list) and all(torch.is_tensor(v) for v in value):
                values.append((name, sum(v.mean() for v in value)))
            else:
                raise TypeError(f'{name} is not a tensor or list of tensors')
        total = sum(value for name, value in values if 'loss' in name)
        values.insert(0, ('loss', total))
        # detach only the log copies, never the tensor used for backward.
        logged = torch.stack([value.detach() for _, value in values]).cpu().tolist()
        return total, OrderedDict((name, value) for (name, _), value in zip(values, logged))
