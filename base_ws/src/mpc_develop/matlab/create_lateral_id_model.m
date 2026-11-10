function model = create_lateral_id_model(vehicle, tireInitial)
%CREATE_LATERAL_ID_MODEL Build the idnlgrey model for lateral tire fitting.
%
% With no inputs, uses the documented nominal IONIQ 5-sized vehicle values:
%   m = 2100 kg, Iz = 4200 kg*m^2, lf = lr = 1.50 m.
% The 3.00 m wheelbase is from configs/vehicle/ioniq5.yaml. Mass, inertia,
% and CG split are engineering nominal values, not MORAI ground truth.
%
% vehicle must provide fixed, positive SI values:
%   m [kg], Iz [kg*m^2], lf [m], lr [m]
%
% tireInitial must provide positive initial estimates:
%   Bf, Cf, Df, Br, Cr, Dr
%
% The vehicle values are intentionally fixed. Estimating mass/inertia and
% Pacejka peak factors together is poorly identifiable from lateral data.

if nargin < 1 || isempty(vehicle)
    vehicle = nominalVehicle();
end
if nargin < 2 || isempty(tireInitial)
    tireInitial = nominalTireInitial();
end

vehicleNames = {"m", "Iz", "lf", "lr"};
tireNames = {"Bf", "Cf", "Df", "Br", "Cr", "Dr"};

vehicleValues = valuesFromStruct(vehicle, vehicleNames, "vehicle");
tireValues = valuesFromStruct(tireInitial, tireNames, "tireInitial");

parameterValues = [vehicleValues; tireValues];
parameterNames = [vehicleNames, tireNames];
parameterUnits = {"kg", "kg*m^2", "m", "m", "1/rad", "1", "N", ...
                  "1/rad", "1", "N"};

% Order = [number of outputs, number of inputs, number of states].
% y = [r, ay, vy], u = [delta, vx], x = [vy, r].
model = idnlgrey("dynamic_bicycle_pacejka_e0", [3 2 2], ...
    parameterValues, [0; 0], 0, Name="Lateral dynamic bicycle, Pacejka E=0");

for index = 1:numel(parameterNames)
    model.Parameters(index).Name = char(parameterNames{index});
    model.Parameters(index).Unit = char(parameterUnits{index});
    model.Parameters(index).Value = parameterValues(index);
    if index <= numel(vehicleNames)
        model.Parameters(index).Fixed = true;
    else
        model.Parameters(index).Fixed = false;
        model.Parameters(index).Minimum = eps;
        model.Parameters(index).Maximum = Inf;
    end
end

model.InputName = {'delta'; 'vx'};
model.InputUnit = {'rad'; 'm/s'};
model.OutputName = {'yaw_rate'; 'ay'; 'vy'};
model.OutputUnit = {'rad/s'; 'm/s^2'; 'm/s'};

model.InitialStates(1).Name = 'vy0';
model.InitialStates(1).Unit = 'm/s';
model.InitialStates(1).Fixed = false;
model.InitialStates(2).Name = 'r0';
model.InitialStates(2).Unit = 'rad/s';
model.InitialStates(2).Fixed = false;
end

function vehicle = nominalVehicle()
% IONIQ 5-sized EV nominal model. Wheelbase is repository configuration;
% the other values are deliberately conservative initial assumptions.
vehicle = struct( ...
    'm', 2100, ...     % kg
    'Iz', 4200, ...    % kg*m^2
    'lf', 1.50, ...    % m
    'lr', 1.50);       % m
end

function tireInitial = nominalTireInitial()
% Per-axle initial force scale close to half of a 2100 kg vehicle's weight.
% These are starting values only; nlgreyest estimates all six B/C/D values.
tireInitial = struct( ...
    'Bf', 10.0, 'Cf', 1.30, 'Df', 10000, ...
    'Br', 10.0, 'Cr', 1.30, 'Dr', 10000);
end

function values = valuesFromStruct(source, names, sourceName)
values = zeros(numel(names), 1);
for index = 1:numel(names)
    name = names{index};
    if ~isfield(source, name)
        error("mpc_develop:MissingParameter", ...
            "%s.%s is required.", sourceName, name);
    end
    value = source.(name);
    if ~(isnumeric(value) && isscalar(value) && isfinite(value) && value > 0)
        error("mpc_develop:InvalidParameter", ...
            "%s.%s must be a finite positive scalar.", sourceName, name);
    end
    values(index) = value;
end
end
