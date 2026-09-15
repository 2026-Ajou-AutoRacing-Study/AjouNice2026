# Team experiment YAML

Only the schema and this example are tracked. Real experiment files are local
and ignored so a long-running container is not coupled to Git branch changes.

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
