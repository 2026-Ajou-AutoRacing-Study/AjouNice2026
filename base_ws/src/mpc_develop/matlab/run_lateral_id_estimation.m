%% 1. 모델 코드와 CSV 폴더 지정
clear; clc; close all;

addpath('/home/kdy/AjouNice2026/base_ws/src/mpc_develop/matlab');

csvFolder = ...
    '/home/kdy/AjouNice2026/planner_logs/analysis/mpc_develop/imported_recent_20261108_1417';

files = dir(fullfile(csvFolder, '*_lateral_id.csv'));
assert(~isempty(files), 'CSV 파일을 찾지 못했습니다.');

Ts = 0.05;          % CSV 변환 시 사용한 20 Hz
minSamples = 40;    % 너무 짧은 구간 제거: 2초

%% 2. bag 단위로 추정용 / 검증용 분리
rng(7);  % 매번 같은 분리 결과 사용
fileOrder = randperm(numel(files));
numberOfEstimationBags = ceil(0.7 * numel(files));

isEstimationBag = false(numel(files), 1);
isEstimationBag(fileOrder(1:numberOfEstimationBags)) = true;

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
% create_lateral_id_model.m 안의 명목 차량값을 사용
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
resid(zVal, estimatedModel);

present(estimatedModel)
