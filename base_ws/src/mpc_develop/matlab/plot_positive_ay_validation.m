%% Measured ay >= +6 m/s^2, with two seconds of context on each side.
addpath(fileparts(mfilename('fullpath')));
[zValAll, baseline] = load_baseline_validation();
threshold = 6;  % m/s^2; applied to measured ay, not predicted ay
contextSeconds = 2;
eventCount = 0;
for experimentIndex = 1:numel(zValAll.ExperimentName)
    data = getexp(zValAll, experimentIndex);
    measured = data.OutputData;
    if iscell(measured), measured = measured{1}; end
    indices = find(measured(:,2) >= threshold);
    if isempty(indices), continue; end
    % Expand each sample by the context window, then merge overlaps.
    padding = ceil(contextSeconds / data.Ts);
    starts = max(1, indices - padding);
    ends = min(size(measured,1), indices + padding);
    groupStarts = starts(1);
    groupEnds = ends(1);
    for j = 2:numel(indices)
        if starts(j) <= groupEnds(end) + 1
            groupEnds(end) = max(groupEnds(end), ends(j));
        else
            groupStarts(end+1) = starts(j); %#ok<SAGROW>
            groupEnds(end+1) = ends(j); %#ok<SAGROW>
        end
    end
    for j = 1:numel(groupStarts)
        eventCount = eventCount + 1;
        [fig, result] = plot_lateral_model_comparison(zValAll, baseline, experimentIndex);
        limits = ([groupStarts(j), groupEnds(j)] - 1) * data.Ts;
        axesList = findall(fig, 'Type', 'axes');
        for k = 1:numel(axesList), xlim(axesList(k), limits); end
        fprintf('Experiment %d: %s, %.2f-%.2f s, peak measured ay %.2f m/s^2; fit is for the full experiment.\n', ...
            experimentIndex, result.experimentName, limits(1), limits(2), ...
            max(measured(groupStarts(j):groupEnds(j),2)));
    end
end
if eventCount == 0
    fprintf('No validation samples have measured ay >= +%.2f m/s^2.\n', threshold);
else
    fprintf('Created %d event figure(s).\n', eventCount);
end
