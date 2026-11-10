function [fig, results] = plot_pacejka_e_comparison(zVal, baseline, candidate, experimentIndex)
%PLOT_PACEJKA_E_COMPARISON Compare measurements and E=0/free-E models.
% Each model is evaluated on the same validation experiment using the same
% MATLAB compare defaults, including its initial-condition handling.
if nargin < 4
    experimentIndex = 1;
end
validateattributes(experimentIndex, {'numeric'}, ...
    {'scalar','integer','positive','<=',numel(zVal.ExperimentName)});
data = getexp(zVal, experimentIndex);
[baselineResponse, baselineFit, baselineIC] = compare(data, baseline);
[candidateResponse, candidateFit, candidateIC] = compare(data, candidate);
y = data.OutputData;
yBaseline = baselineResponse.OutputData;
yCandidate = candidateResponse.OutputData;
if iscell(y), y = y{1}; end
if iscell(yBaseline), yBaseline = yBaseline{1}; end
if iscell(yCandidate), yCandidate = yCandidate{1}; end
if iscell(baselineFit), baselineFit = baselineFit{1}; end
if iscell(candidateFit), candidateFit = candidateFit{1}; end
if size(y,2) ~= 3 || ~isequal(size(y),size(yBaseline),size(yCandidate))
    error('mpc_develop:InvalidComparisonData', 'Expected matched yaw rate, ay and vy outputs.');
end

parameterNames = {candidate.Parameters.Name};
efIndex = find(strcmp(parameterNames, 'Ef'), 1);
erIndex = find(strcmp(parameterNames, 'Er'), 1);
if isempty(efIndex) || isempty(erIndex)
    error('mpc_develop:MissingCurvature', 'Candidate must contain Ef and Er.');
end
ef = candidate.Parameters(efIndex).Value;
er = candidate.Parameters(erIndex).Value;
time = (0:size(y,1)-1)' * data.Ts;
fig = figure('Name', 'Pacejka E model comparison', 'Color', 'w');
labels = {'Yaw rate [rad/s]', 'Lateral acceleration [m/s^2]', 'Lateral velocity [m/s]'};
for channel = 1:3
    ax = subplot(3,1,channel,'Parent',fig);
    plot(ax,time,y(:,channel),'k','LineWidth',1.2,'DisplayName','GT / measurement');
    hold(ax,'on');
    plot(ax,time,yBaseline(:,channel),'b','LineWidth',1.0, ...
        'DisplayName',sprintf('E = 0: %.1f%%',baselineFit(channel)));
    plot(ax,time,yCandidate(:,channel),'r','LineWidth',1.0, ...
        'DisplayName',sprintf('Estimated E: %.1f%%',candidateFit(channel)));
    grid(ax,'on'); ylabel(ax,labels{channel}); legend(ax,'Location','best');
    if channel == 3, xlabel(ax,'Time [s]'); end
end
sgtitle(fig,{sprintf('Validation experiment %d: %s',experimentIndex,data.ExperimentName{1}), ...
    sprintf('Candidate: Ef = %.4g, Er = %.4g',ef,er)},'Interpreter','none','FontSize',10);
results = struct('experimentName',data.ExperimentName{1},'time',time, ...
    'measured',y,'baseline',yBaseline,'candidate',yCandidate, ...
    'baselineFit',baselineFit,'candidateFit',candidateFit, ...
    'baselineInitialConditions',baselineIC,'candidateInitialConditions',candidateIC, ...
    'Ef',ef,'Er',er);
end
