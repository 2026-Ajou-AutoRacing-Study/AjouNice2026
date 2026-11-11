%% Plot every validation segment against the saved CG baseline.
addpath(fileparts(mfilename('fullpath')));
[zValAll, baseline] = load_baseline_validation();
for i = 1:numel(zValAll.ExperimentName)
    fprintf('%d: %s\n', i, zValAll.ExperimentName{i});
    plot_lateral_model_comparison(zValAll, baseline, i);
end
