# Planner CSV Logger

`planner_csv_logger.py` samples the latest planner and vehicle state into one
CSV row at a configurable rate. Nested stop reasons, planning factors, and
ROS logs are stored as JSON strings inside CSV cells.

The K-City MORAI planner RViz includes a `Planner CSV Logger` panel. Press
`Start Logging` to create the next numbered file and `Stop Logging` to flush
and close it. Pressing Start again creates the following number.

The logger includes:

- BPP and BVP module status;
- stop reasons and planning factors;
- localization pose, velocity, and acceleration;
- summarized trajectories at every velocity-planning boundary: BVP path, path
  smoother path, path optimizer trajectory, motion velocity planner raw
  trajectory, and final control trajectory;
- MORAI control command and ego status;
- operation mode and recent `/rosout_agg` messages;
- Git revision and the Behavior Velocity Planner parameter snapshot.

The path summaries include the message stamp and receive age, point count, the
point nearest to ego, the minimum and maximum velocity ahead, and the first
zero-velocity point ahead of ego (index, distance, and map position). Trajectory
messages also include the acceleration at the point nearest to ego. This makes
it possible to identify the exact pipeline boundary where a stop target is
lost without writing every trajectory point at every sample.

## Run

The normal planner RViz launch starts the logger node in its idle state:

```bash
roslaunch bringup kcity_morai_competition_planner_rviz.launch
```

Use the Start and Stop buttons in the RViz panel. To run without RViz and start
recording immediately, use:

```bash
roslaunch behavior_velocity_planner planner_csv_logger.launch auto_start:=true
```

The logger creates files under `$AJOUNICE_ROOT/planner_logs` without requiring a
filename. It scans existing files and increments the largest number:

```text
log_data_01.csv
log_data_02.csv
log_data_03.csv
```

The optional `output_dir` argument changes only the directory; filenames are
always assigned automatically.

Stop the logger with `Ctrl+C`. The file is flushed after every row by default.

Useful arguments:

```text
rate_hz:=10.0
rosout_min_level:=INFO
scenario_note:="free-form note"
```

All topic names can be overridden as private parameters below
`~topics/<key>`.
