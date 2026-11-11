function [fig, results] = plot_lateral_model_comparison(zVal, dynamicModel, experimentIndex)
%PLOT_LATERAL_MODEL_COMPARISON Compare measured and dynamic model outputs.
% zVal inputs: [front-wheel angle, body longitudinal velocity].
% Outputs: [yaw rate, body lateral acceleration, body lateral velocity].
% The dynamic model outputs CG lateral velocity.

if nargin < 3
    experimentIndex = 1;
end
validateattributes(experimentIndex, {'numeric'}, ...
    {'scalar', 'integer', 'positive', '<=', numel(zVal.ExperimentName)});
data = getexp(zVal, experimentIndex);
u = data.InputData;
y = data.OutputData;
Ts = data.Ts;
if size(u, 2) ~= 2 || size(y, 2) ~= 3 || size(u, 1) < 3 || ...
        ~isscalar(Ts) || ~isfinite(Ts) || Ts <= 0
    error('mpc_develop:InvalidComparisonData', ...
        'Expected at least 3 samples, 2 inputs, 3 outputs, and a positive sample time.');
end

names = {dynamicModel.Parameters.Name};
% Use the values in the supplied model object, whether it is estimatedModel
% or an initial model. Do not label them as estimated without that context.
tireNames = {'Bf', 'Cf', 'Df', 'Br', 'Cr', 'Dr'};
tireParameters = struct();
for index = 1:numel(tireNames)
    parameterIndex = find(strcmp(names, tireNames{index}), 1);
    if isempty(parameterIndex)
        error('mpc_develop:MissingTireParameter', ...
            'Dynamic model is missing %s.', tireNames{index});
    end
    tireParameters.(tireNames{index}) = dynamicModel.Parameters(parameterIndex).Value;
end
% Match the existing compare() simulation and initial-condition handling.
[dynamicResponse, dynamicFit, initialConditions] = compare(data, dynamicModel);
yDynamic = dynamicResponse.OutputData;
if iscell(yDynamic)
    yDynamic = yDynamic{1};
end
if iscell(dynamicFit)
    dynamicFit = dynamicFit{1};
end
time = (0:size(y, 1)-1)' * Ts;
fig = figure('Name', 'Lateral model comparison');
labels = {'Yaw rate [rad/s]', 'Lateral acceleration [m/s^2]', 'Lateral velocity [m/s]'};
for channel = 1:3
    ax = subplot(3, 1, channel, 'Parent', fig);
    plot(ax, time, y(:, channel), 'k', 'LineWidth', 1.2, ...
        'DisplayName', 'GT / measurement');
    hold(ax, 'on');
    plot(ax, time, yDynamic(:, channel), 'r', 'LineWidth', 1.0, ...
        'DisplayName', sprintf('Dynamic Pacejka: %.1f%%', dynamicFit(channel)));
    grid(ax, 'on');
    ylabel(ax, labels{channel});
    legend(ax, 'Location', 'best');
    if channel == 3
        xlabel(ax, 'Time [s]');
    end
end
efIndex = find(strcmp(names, 'Ef'), 1);
erIndex = find(strcmp(names, 'Er'), 1);
if isempty(efIndex) && isempty(erIndex)
    tireParameters.Ef = 0; tireParameters.Er = 0;
elseif ~isempty(efIndex) && ~isempty(erIndex)
    tireParameters.Ef = dynamicModel.Parameters(efIndex).Value;
    tireParameters.Er = dynamicModel.Parameters(erIndex).Value;
else
    error('mpc_develop:MissingTireParameter', 'Model must define both Ef and Er.');
end
sgtitle(fig, {sprintf('Validation experiment %d: %s', ...
    experimentIndex, data.ExperimentName{1}), ...
    sprintf('Pacejka: Front Bf=%.4g, Cf=%.4g, Df=%.4g N, Ef=%.4g', ...
    tireParameters.Bf, tireParameters.Cf, tireParameters.Df, tireParameters.Ef), ...
    sprintf('Rear Br=%.4g, Cr=%.4g, Dr=%.4g N, Er=%.4g', ...
    tireParameters.Br, tireParameters.Cr, tireParameters.Dr, tireParameters.Er)}, ...
    'Interpreter', 'none', 'FontSize', 10);
results = struct('time', time, 'measured', y, ...
    'dynamic', yDynamic, ...
    'dynamicFit', dynamicFit, 'dynamicInitialConditions', initialConditions, ...
    'experimentName', data.ExperimentName{1}, 'tireParameters', tireParameters);
end
