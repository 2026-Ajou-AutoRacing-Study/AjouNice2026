function [zValAll, baseline] = load_baseline_validation()
% Load the original and new validation segments with the saved CG model.
baseDir = '/home/kdy/AjouNice2026/planner_logs/analysis/mpc_develop/pacejka_e_comparison_20261108_trial';
newDir = '/home/kdy/AjouNice2026/planner_logs/analysis/mpc_develop/imported_20261110';
savedData = load(fullfile(baseDir, 'data.mat'), 'zVal');
savedModel = load(fullfile(baseDir, 'baseline.mat'), 'baseline');
zValAll = savedData.zVal;
baseline = savedModel.baseline;
names = {
    'planbag_pv_highway_S6_70_S_23254497_P_20261102_025845_lateral_id.csv'
    'planbag_scenario_078_sunny_intersection_1obj_P_20261102_065126_lateral_id.csv'
};
for i = 1:numel(names)
    T = readtable(fullfile(newDir, names{i}));
    for segmentId = unique(T.segment_id(:))'
        rows = T.segment_id == segmentId;
        if nnz(rows) < 40, continue; end
        u = [T.delta_rad(rows), T.vx_mps(rows)];
        y = [T.yaw_rate_radps(rows), T.ay_mps2(rows), T.vy_mps(rows)];
        experiment = iddata(y, u, 0.05);
        experiment.InputName = zValAll.InputName;
        experiment.InputUnit = zValAll.InputUnit;
        experiment.OutputName = zValAll.OutputName;
        experiment.OutputUnit = zValAll.OutputUnit;
        experiment.ExperimentName = {sprintf('%s_seg%d', ...
            erase(names{i}, '_lateral_id.csv'), segmentId)};
        zValAll = merge(zValAll, experiment);
    end
end
end
