"""Restore config geometry AFTER checkpoint loading, before train/val/test."""
from mmengine.hooks import Hook
from mmdet3d.registry import HOOKS
from perception.detection3d.bevfusion.view_geometry import restore_view_geometry


@HOOKS.register_module()
class MoraiConfigViewGeometryHook(Hook):
    def _restore(self, runner):
        model = runner.model
        while hasattr(model, 'module'):
            model = model.module
        restore_view_geometry(model.view_transform)

    def before_train(self, runner):
        self._restore(runner)

    def before_val(self, runner):
        self._restore(runner)

    def before_test(self, runner):
        self._restore(runner)
