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

For identification, remove standstill/low-speed samples before forming
`iddata`. The model has a numerical `vx` guard only to avoid division by zero;
it does not make a dynamic-bicycle model valid at low speed.

After estimation, `run_lateral_id_estimation.m` plots measured outputs in
black, a no-slip kinematic bicycle in blue, and the identified dynamic model
in red. Change `validationExperiment` and rerun section 8, or call:

```matlab
[fig, results] = plot_lateral_model_comparison(zVal, estimatedModel, 2);
```

The figure title shows the six tire parameter values from the model object
passed to the function. With `estimatedModel` these are the estimated values;
with `initialModel` they are initial guesses. `results.tireParameters` contains
the same full-precision values.

The kinematic baseline uses the identified model's `lf` and `lr` and the same
measured steering and body longitudinal velocity. Its lateral acceleration
uses the numerical derivative of lateral velocity and can amplify measurement
noise. Its fit values use the same normalized error formula as `compare`;
the dynamic response uses MATLAB `compare` initial-condition handling.
The black curve is recorded measurement, not necessarily noise-free ground truth.

## Experimental Pacejka E comparison

`run_pacejka_e_comparison` uses the same bag split and 20 Hz data as
`run_lateral_id_estimation.m`. It fits a fresh E=0 baseline, then one
candidate initialized from that baseline with Ef/Er = 0.
Both E parameters use experimental bounds [-2,1]. The output folder contains
the split, data, fitted models, parameters, and per-experiment validation fit
and RMSE. Selecting E based on validation data makes that data a model-selection
set; use independent bag(s) for a final unbiased assessment.

```matlab
run_pacejka_e_comparison  % creates a timestamped analysis output folder
% After loading a candidate and data.mat from that folder:
plot_lateral_model_comparison(zVal, candidate, 1)
```

To compare the measured signals (black), E=0 baseline (blue), and free-E
candidate (red) directly on a single validation experiment:

```matlab
plot_pacejka_e_comparison(zVal, baseline, candidate, 5)
```

Both models use MATLAB `compare`'s default initial-condition handling. They
were not guaranteed to converge in the 20-iteration trial, and the validation
bags are a model-selection set, not a final independent test.

The studies assume the existing CSV steering units, velocity frame, and IMU
conventions. They do not establish physical tire parameters or fix those
unverified signal assumptions. E=0 and free-E models share the same nominal
fixed vehicle mass, yaw inertia, and axle distances.

## Linear tire force comparison

`dynamic_bicycle_linear_tire.m` keeps the current bicycle state equations,
exact slip-angle geometry, and input/output order but replaces the Pacejka
force by `FyF = CalphaF*alphaF`, `FyR = CalphaR*alphaR`. The two positive
cornering stiffness parameters are per axle in N/rad and are estimated with
`nlgreyest`. The default starting value is 130000 N/rad for each axle; this
is a guess, not a measured quantity. The model is not globally linear because
it retains atan2, cos, and measured time-varying vx.

Run `run_linear_tire_estimation.m` to use the same bag-separated data and
vehicle assumptions as the existing E=0 experiment. The model and comparison
remain in the MATLAB workspace; inspect solver termination and compare all
held-out experiments before drawing conclusions.
