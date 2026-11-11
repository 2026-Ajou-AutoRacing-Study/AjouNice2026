%% 1. 모델 코드와 CSV 폴더 지정
clear; clc; close all;

addpath('/home/kdy/AjouNice2026/base_ws/src/mpc_develop/matlab');

csvFolder = ...
    '/home/kdy/AjouNice2026/planner_logs/analysis/mpc_develop/imported_recent_20261108_1417';

files = dir(fullfile(csvFolder, '*_lateral_id.csv'));
assert(numel(files) == 12, '기존 CSV가 12개가 아닙니다. 폴더 내용을 확인하세요.');

% 새 파일 중 기존 학습 데이터와 중복되지 않는 세 개만 추가한다.
newFolder = ...
    '/home/kdy/AjouNice2026/planner_logs/analysis/mpc_develop/imported_20261110';
newNames = {
    'planbag_pv_highway_S6_70_S_23254497_P_20261102_025845_lateral_id.csv'
    'planbag_scenario_078_sunny_intersection_1obj_P_20261102_065126_lateral_id.csv'
    'planbag_rb_150_T5_S0_V22_O0_P_20261102_040935_lateral_id.csv'
};
for i = 1:numel(newNames)
    newFile = dir(fullfile(newFolder, newNames{i}));
    assert(numel(newFile) == 1, '새 CSV를 찾지 못했습니다: %s', newNames{i});
    files(end+1) = newFile;
end

Ts = 0.05;          % CSV 변환 시 사용한 20 Hz
minSamples = 40;    % 너무 짧은 구간 제거: 2초

%% 2. 기존 검증 bag과 새 검증 bag 두 개를 고정
validationNames = {
    'planbag_full_source_mix_037_P_20261031_231526_lateral_id.csv'
    'planbag_pv_highway_S6_70_S_57f13556_P_20261102_030146_lateral_id.csv'
    'planbag_scenario_197_sunny_intersection_1obj_P_20261102_084142_lateral_id.csv'
    'planbag_pv_highway_S6_70_S_23254497_P_20261102_025845_lateral_id.csv'
    'planbag_scenario_078_sunny_intersection_1obj_P_20261102_065126_lateral_id.csv'
};
isValidationBag = ismember({files.name}, validationNames);
assert(nnz(isValidationBag) == numel(validationNames), '검증용 CSV를 모두 찾지 못했습니다.');
isEstimationBag = ~isValidationBag;
fprintf('추정용 CSV: %d개\n', nnz(isEstimationBag));
fprintf('검증용 CSV: %d개\n', nnz(isValidationBag));

uEst = {};
yEst = {};
nameEst = {};

uVal = {};
yVal = {};
nameVal = {};

%% 3. 각 CSV의 segment_id를 각각 독립 실험으로 추가
for fileIndex = 1:numel(files)
    csvPath = fullfile(files(fileIndex).folder, files(fileIndex).name);
    T = readtable(csvPath);

    segmentIds = unique(T.segment_id);

    for segmentId = segmentIds'
        rows = T.segment_id == segmentId;

        if nnz(rows) < minSamples
            continue;
        end

        % 기존 lateral model의 정확한 입·출력 순서
        u = [T.delta_rad(rows), T.vx_mps(rows)];
        y = [T.yaw_rate_radps(rows), ...
             T.ay_mps2(rows), ...
             T.vy_mps(rows)];

        experimentName = sprintf('%s_seg%d', ...
            erase(files(fileIndex).name, '_lateral_id.csv'), segmentId);

        if isEstimationBag(fileIndex)
            uEst{end+1,1} = u;
            yEst{end+1,1} = y;
            nameEst{end+1,1} = experimentName;
        else
            uVal{end+1,1} = u;
            yVal{end+1,1} = y;
            nameVal{end+1,1} = experimentName;
        end
    end
end

%% 4. MATLAB System Identification 데이터 객체 생성
zEst = iddata(yEst, uEst, Ts);
zEst.InputName = {'delta', 'vx'};
zEst.InputUnit = {'rad', 'm/s'};
zEst.OutputName = {'yaw_rate', 'ay', 'vy'};
zEst.OutputUnit = {'rad/s', 'm/s^2', 'm/s'};
zEst.ExperimentName = nameEst;

zVal = iddata(yVal, uVal, Ts);
zVal.InputName = zEst.InputName;
zVal.InputUnit = zEst.InputUnit;
zVal.OutputName = zEst.OutputName;
zVal.OutputUnit = zEst.OutputUnit;
zVal.ExperimentName = nameVal;

%% 5. 추정 전에 부호·응답 확인
figure;
plot(zEst);
sgtitle('Estimation data: delta, vx, yaw rate, ay, vy');

%% 6. 기존 Pacejka E=0 모델 생성
initialModel = create_lateral_id_model();

%% 7. 타이어 B/C/D 파라미터 추정
options = nlgreyestOptions( ...
    'Display', 'on', ...
    'SearchMethod', 'lsqnonlin');

estimatedModel = nlgreyest(zEst, initialModel, options);

%% 8. 한 번도 추정에 쓰지 않은 bag으로 검증
% 검증 experiment 번호를 바꿔 각 구간을 비교한다.
validationExperiment = 1;
plot_lateral_model_comparison(zVal, estimatedModel, validationExperiment);

figure;
% Fit used many experiments, so the model stores one initial-state set per
% training experiment. Analyze the selected validation experiment separately
% with zero initial states rather than reusing the training initial states.
% This residual plot can be affected by initial transients.
resid(getexp(zVal, validationExperiment), estimatedModel, ...
    residOptions('InitialCondition', 'z'));

present(estimatedModel)
