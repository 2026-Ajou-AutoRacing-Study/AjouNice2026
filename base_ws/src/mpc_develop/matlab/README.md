# MATLAB lateral tire-identification model

This folder contains the continuous-time, two-state lateral dynamic bicycle
model used to identify simplified Pacejka tire parameters with `nlgreyest`.

```matlab
cd /home/kdy/AjouNice2026/base_ws/src/mpc_develop/matlab
addpath(pwd)
```

For a first run, the model can be created without arguments. This uses an
IONIQ 5-sized nominal vehicle: `m=2100 kg`, `Iz=4200 kg*m^2`, and a symmetric
CG split `lf=lr=1.50 m`. The configured 3.00 m wheelbase is preserved. Only
the wheelbase is repository configuration; mass, inertia, and CG split are
engineering nominal assumptions and must be treated as such.

```matlab
initialModel = create_lateral_id_model();
```

Alternatively, supply a different fixed vehicle model and positive initial
guesses for tire parameters. Initial guesses are not identified values;
`nlgreyest` changes the six free tire parameters.

```matlab
vehicle = struct( ...
    'm',  vehicle_mass_kg, ...
    'Iz', vehicle_yaw_inertia_kgm2, ...
    'lf', cg_to_front_axle_m, ...
    'lr', cg_to_rear_axle_m);

tireInitial = struct( ...
    'Bf', initial_Bf, 'Cf', initial_Cf, 'Df', initial_Df, ...
    'Br', initial_Br, 'Cr', initial_Cr, 'Dr', initial_Dr);

initialModel = create_lateral_id_model(vehicle, tireInitial);
```

The model contract is:

```matlab
% u = [delta, vx]          % [rad, m/s]
% y = [yaw_rate, ay, vy]   % [rad/s, m/s^2, m/s]
% x = [vy, yaw_rate]
```

The model outputs CG lateral velocity. The reference point and axis convention
of the recorded Ego velocity must be verified before treating it as CG ground truth.

For identification, remove standstill/low-speed samples before forming
`iddata`. The model has a numerical `vx` guard only to avoid division by zero;
it does not make a dynamic-bicycle model valid at low speed.

After estimation, `run_lateral_id_estimation.m` plots measured outputs in
black and the identified dynamic model in red. Change `validationExperiment` and rerun section 8, or call:

```matlab
[fig, results] = plot_lateral_model_comparison(zVal, estimatedModel, 2);
```

The figure title shows the six tire parameter values from the model object
passed to the function. With `estimatedModel` these are the estimated values;
with `initialModel` they are initial guesses. `results.tireParameters` contains
the same full-precision values.

The dynamic response uses MATLAB `compare` initial-condition handling.
The black curve is recorded measurement, not necessarily noise-free ground truth.

## Ego velocity reference diagnostic (experimental)

`extract_ego_imu_csv.py` exports raw Ego position/velocity/heading and IMU
samples from a ROS1 bag into separate `ego.csv` and `imu.csv`; the lateral ID
CSV alone has no positions and cannot answer the reference-point question.
The extractor preserves original topic timestamps and refuses to overwrite.

```matlab
d = '/home/kdy/AjouNice2026/planner_logs/analysis/mpc_develop/ego_reference_037';
r = diagnose_ego_reference(fullfile(d,'ego.csv'),fullfile(d,'imu.csv'));
```

This assumes position is rear-axle, world heading and x-forward/y-left;
none is established by the computation. Heading convention, offsets, sensor
timestamps, filtering, actual sideslip, road gradient, and outliers can bias
results. The angle-based world/body selection assumes small sideslip; its
choice is not proof. The one-variable regressions and heading shift are
correlated explanations, not causal identification. Only the longest continuous
(overlap gap <= 0.15 s) section is analyzed; filter edges are excluded from
fits. Results from a validation bag cannot establish an independent final test.

`create_reference_id_model(cfg, baseline)` is opt-in. `cfg.vySource` defaults
to `velocity`, `cfg.x_ego=NaN` retains CG output, `cfg.x_imu=1.528`,
`cfg.y_imu=0`, and `cfg.applyImuOffset=false` preserves the original ay.
With a finite x_ego the vy output is corrected; set applyImuOffset true to
also correct ay. The longitudinal geometry uses L-lf. `initial_vy_cg` performs
the initial reference conversion. `compare_reference_fit` initializes each
training experiment from measured vy/r and estimates free initial states.
Position-derivative fitting requires raw ego/imu exports for every bag. Run
`diagnose_ego_reference` for each, convert each with `position_reference_iddata`,
then merge experiments into bag-separated zEst and zVal. Set `vySource` to
`position_derivative` and `positionDataPrepared=true`; x_ego is fixed at zero.
This path cannot consume the older position-free lateral ID CSVs.
