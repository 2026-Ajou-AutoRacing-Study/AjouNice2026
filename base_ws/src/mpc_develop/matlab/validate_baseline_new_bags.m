%% 기존 CG 모델을 새로운 두 bag으로 검증한다. 재추정은 수행하지 않는다.
modelFolder = fileparts(mfilename('fullpath'));
addpath(modelFolder);
baselinePath = '/home/kdy/AjouNice2026/planner_logs/analysis/mpc_develop/pacejka_e_comparison_20261108_trial/baseline.mat';
newFolder = '/home/kdy/AjouNice2026/planner_logs/analysis/mpc_develop/imported_20261110';
names = {
    'planbag_pv_highway_S6_70_S_23254497_P_20261102_025845_lateral_id.csv'
    'planbag_scenario_078_sunny_intersection_1obj_P_20261102_065126_lateral_id.csv'
};
loaded = load(baselinePath, 'baseline');
baseline = loaded.baseline;
for fileIndex = 1:numel(names)
    T = readtable(fullfile(newFolder, names{fileIndex}));
    segmentIds = unique(T.segment_id);
    for segmentId = segmentIds(:)'
        rows = T.segment_id == segmentId;
        if nnz(rows) < 40, continue; end
        u = [T.delta_rad(rows), T.vx_mps(rows)];
        y = [T.yaw_rate_radps(rows), T.ay_mps2(rows), T.vy_mps(rows)];
        validationData = iddata(y, u, 0.05);
        validationData.InputName = {'delta', 'vx'};
        validationData.OutputName = {'yaw_rate', 'ay', 'vy'};
        validationData.ExperimentName = {sprintf('%s_seg%d', ...
            erase(names{fileIndex}, '_lateral_id.csv'), segmentId)};
        plot_lateral_model_comparison(validationData, baseline, 1);
    end
end
