# Competition Rule Monitor

Development-time observer for the 2026 K-City competition. It never publishes
vehicle commands. One launch starts the judging core and a compact Qt GUI:

```bash
roslaunch competition_rule_monitor competition_rule_monitor.launch
```

The run begins on an observed MORAI `P -> D` transition. CP1 through CP15 must
be passed in order, followed by the separate START/FINISH zone. RViz markers are
published on `/competition/rule_monitor/checkpoints` for the existing planviz.
Speed and judging pose both come from `/Ego_topic`; its `morai_map` pose is
transformed to `map` before lanelet and checkpoint tests.

Outputs are written below `artifacts/competition_rule_monitor/<run timestamp>/`:

- `events.jsonl`: detailed state, checkpoint, contact and penalty events
- `samples.csv`: sampled pose, speed and rule-region state
- `summary.yaml`: final time and penalty totals

Traffic-signal judging is intentionally out of scope because the current stack
does not provide an authoritative judging signal topic.
