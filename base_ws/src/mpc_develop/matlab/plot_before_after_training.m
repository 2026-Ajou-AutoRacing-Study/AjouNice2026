function [fig, results] = plot_before_after_training(zVal, beforeModel, afterModel, experimentIndex)
% Compare both models against the same validation experiment.
if nargin < 4, experimentIndex = 1; end
validateattributes(experimentIndex, {'numeric'}, ...
    {'scalar', 'integer', 'positive', '<=', numel(zVal.ExperimentName)});
data = getexp(zVal, experimentIndex);
[before, beforeFit] = compare(data, beforeModel);
[after, afterFit] = compare(data, afterModel);
y = data.OutputData;
yBefore = before.OutputData;
yAfter = after.OutputData;
if iscell(y), y = y{1}; end
if iscell(yBefore), yBefore = yBefore{1}; end
if iscell(yAfter), yAfter = yAfter{1}; end
if iscell(beforeFit), beforeFit = beforeFit{1}; end
if iscell(afterFit), afterFit = afterFit{1}; end
assert(size(y,2) == 3 && isequal(size(y),size(yBefore),size(yAfter)), ...
    'Expected three matching output channels.');
t = (0:size(y,1)-1)' * data.Ts;
labels = {'Yaw rate [rad/s]', 'Lateral acceleration [m/s^2]', 'Lateral velocity [m/s]'};
fig = figure('Name','Before and after additional training','Color','w');
for k = 1:3
    ax = subplot(3,1,k,'Parent',fig);
    plot(ax,t,y(:,k),'k','LineWidth',1.2,'DisplayName','GT / measurement');
    hold(ax,'on');
    plot(ax,t,yBefore(:,k),'b','DisplayName',sprintf('Before: %.1f%%',beforeFit(k)));
    plot(ax,t,yAfter(:,k),'r','DisplayName',sprintf('After: %.1f%%',afterFit(k)));
    grid(ax,'on'); ylabel(ax,labels{k}); legend(ax,'Location','best');
    if k == 3, xlabel(ax,'Time [s]'); end
end
sgtitle(fig,sprintf('Validation experiment %d: %s',experimentIndex, ...
    data.ExperimentName{1}),'Interpreter','none');
results = struct('experimentName',data.ExperimentName{1}, ...
    'beforeFit',beforeFit,'afterFit',afterFit);
end
