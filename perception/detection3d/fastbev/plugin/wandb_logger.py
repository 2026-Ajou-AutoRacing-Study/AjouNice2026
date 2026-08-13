"""W&B logging conventions for MORAI Fast-BEV experiments."""

from mmcv.runner import HOOKS
from mmcv.runner.dist_utils import master_only
from mmcv.runner.hooks.logger import WandbLoggerHook


@HOOKS.register_module()
class MoraiWandbLoggerHook(WandbLoggerHook):
    """Log explicit epoch/global-step axes without changing Fast-BEV's loop."""

    @master_only
    def before_run(self, runner):
        super().before_run(runner)
        # Training scalars remain iteration-resolved while per-epoch
        # validation scalars get a natural epoch axis in the W&B UI.  Detailed
        # metrics are hidden only from automatic panel generation: their full
        # history remains available for manually grouped dashboard panels.
        self.wandb.define_metric("global_step", hidden=True)
        self.wandb.define_metric("epoch", hidden=True)
        self.wandb.define_metric("train/*", step_metric="global_step")
        self.wandb.define_metric(
            "val/*", step_metric="epoch", hidden=True)
        for name in (
                "val/center_mAP",
                "val/center/vehicle_mean_AP",
                "val/center/pedestrian_mean_AP",
                "val/center/obstacle_mean_AP"):
            self.wandb.define_metric(
                name,
                step_metric="epoch",
                hidden=False,
                summary="max",
                overwrite=True)
        self.wandb.define_metric(
            "learning_rate", step_metric="global_step")
        self.wandb.define_metric(
            "momentum", step_metric="global_step")

    def get_loggable_tags(self, runner, *args, **kwargs):
        tags = super().get_loggable_tags(runner, *args, **kwargs)
        tags["epoch"] = runner.epoch + 1
        tags["global_step"] = runner.iter + 1
        return tags
