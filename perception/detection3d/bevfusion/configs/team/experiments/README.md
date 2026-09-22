# Team experiment YAML

Only the schema and this example are tracked. Real experiment files are local
and ignored so a long-running container is not coupled to Git branch changes.

Reviewed team recipes that must be identical across operators are tracked under
`../recipes/`. In particular, the Base3-Online highway fresh and E25 fine-tune
recipes and the Base4 highway V1/T1 reproduction recipes live there; pass their repository-relative YAML path to
`run_team_training.sh`.

Create the local templates once:

```bash
bash perception/detection3d/bevfusion/scripts/configure_team_machine.sh \
  --operator sw
```

Valid operator IDs are `sw`, `yj`, and `jh`. The command creates ignored YAML
under `experiments/<operator>/` and a repository-root
`.env.bevfusion.local`. At launch, the complete input YAML, resolved settings,
MMEngine command, source snapshot, Git state, and runtime manifest are copied
into the output directory and logged to W&B.

Do not put API keys in YAML or `.env.bevfusion.local`.

`checkpointing.max_keep_ckpts` defaults to `3`. Increase it for a long sweep
when intermediate epochs must be compared on rosbag evaluation; for example,
use `40` for a 40-epoch run. Checkpoints are written under the immutable run
output directory, so changing branches does not affect a running experiment.

`schedule.warmup_optimizer_updates` is expressed in optimizer updates. The
runner converts it to MMEngine dataloader iterations using the selected GPU's
gradient accumulation, so recipes remain comparable across team GPUs.
`schedule.validation_interval` controls epoch-based validation frequency.

`dataset.overfit_samples: N` is a diagnostic-only option. It makes train, val,
and test read the same first `N` training samples so a newly added head or
input can be checked for basic learnability. Results from this mode are never
generalization metrics and must not be used to select a production model.
