function model = create_linear_tire_id_model(vehicle, stiffnessInitial)
%CREATE_LINEAR_TIRE_ID_MODEL idnlgrey model with two free axle stiffnesses.
% vehicle fields: m [kg], Iz [kg*m^2], lf and lr [m].
% stiffnessInitial fields: CalphaF and CalphaR [N/rad], per AXLE.
% Defaults use the same nominal vehicle as create_lateral_id_model and
% 130000 N/rad for each axle, B*C*D of its nominal Pacejka initialization.
% Stiffnesses are guesses, not measured or previously identified values.

if nargin < 1 || isempty(vehicle)
    vehicle = struct('m',2100,'Iz',4200,'lf',1.5,'lr',1.5);
end
if nargin < 2 || isempty(stiffnessInitial)
    stiffnessInitial = struct('CalphaF',130000,'CalphaR',130000);
end
names = {'m','Iz','lf','lr','CalphaF','CalphaR'};
units = {'kg','kg*m^2','m','m','N/rad','N/rad'};
values = zeros(1,numel(names));
for k = 1:numel(names)
    if k <= 4, source = vehicle; else, source = stiffnessInitial; end
    if ~isstruct(source) || ~isscalar(source) || ~isfield(source,names{k})
        error('mpc_develop:MissingParameter','Missing scalar structure field %s.',names{k});
    end
    value = source.(names{k});
    if ~(isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value) && value > 0)
        error('mpc_develop:InvalidParameter','%s must be a positive finite real scalar.',names{k});
    end
    values(k) = value;
end

model = idnlgrey('dynamic_bicycle_linear_tire',[3 2 2],values,[0;0],0, ...
    'Name','Lateral dynamic bicycle, linear tire force');
for k = 1:numel(names)
    model.Parameters(k).Name = names{k};
    model.Parameters(k).Unit = units{k};
    model.Parameters(k).Fixed = k <= 4;
    if k > 4
        model.Parameters(k).Minimum = eps;
        model.Parameters(k).Maximum = Inf;
    end
end
model.InputName = {'delta';'vx'};
model.InputUnit = {'rad';'m/s'};
model.OutputName = {'yaw_rate';'ay';'vy'};
model.OutputUnit = {'rad/s';'m/s^2';'m/s'};
model.InitialStates(1).Name = 'vy0'; model.InitialStates(1).Unit = 'm/s';
model.InitialStates(2).Name = 'r0'; model.InitialStates(2).Unit = 'rad/s';
model.InitialStates(1).Fixed = false; model.InitialStates(2).Fixed = false;
end
