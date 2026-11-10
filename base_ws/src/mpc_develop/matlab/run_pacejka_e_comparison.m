function run_pacejka_e_comparison(outputDir, maxIterations)
%RUN_PACEJKA_E_COMPARISON Reproducible bag-separated E=0 versus free-E study.
% E fit starts from the fitted baseline's B/C/D and Ef=Er=0.
% The bounds [-2,1] are experimental, not known simulator parameters.
if nargin < 1
    outputDir = fullfile('/home/kdy/AjouNice2026/planner_logs/analysis/mpc_develop', ...
        ['pacejka_e_comparison_', datestr(now,'yyyymmdd_HHMMSS')]);
end
if nargin < 2, maxIterations = 20; end
if isfolder(outputDir), error('Output directory already exists: %s', outputDir); end
mkdir(outputDir);
csvFolder = '/home/kdy/AjouNice2026/planner_logs/analysis/mpc_develop/imported_recent_20261108_1417';
files = dir(fullfile(csvFolder, '*_lateral_id.csv'));
assert(numel(files) >= 2, 'At least two bags are required.');
Ts = 0.05; minSamples = 40;
rng(7); order = randperm(numel(files));
isEst = false(numel(files),1); isEst(order(1:ceil(0.7*numel(files)))) = true;
uEst = {}; yEst = {}; nameEst = {}; uVal = {}; yVal = {}; nameVal = {};
for j = 1:numel(files)
    T = readtable(fullfile(files(j).folder, files(j).name));
    for segment = unique(T.segment_id)'
        rows = T.segment_id == segment;
        if nnz(rows) < minSamples, continue; end
        u = [T.delta_rad(rows), T.vx_mps(rows)];
        y = [T.yaw_rate_radps(rows), T.ay_mps2(rows), T.vy_mps(rows)];
        name = sprintf('%s_seg%d', erase(files(j).name,'_lateral_id.csv'), segment);
        if isEst(j)
            uEst{end+1,1}=u; yEst{end+1,1}=y; nameEst{end+1,1}=name;
        else
            uVal{end+1,1}=u; yVal{end+1,1}=y; nameVal{end+1,1}=name;
        end
    end
end
zEst = iddata(yEst,uEst,Ts); zVal = iddata(yVal,uVal,Ts);
zEst.InputName={'delta','vx'}; zEst.InputUnit={'rad','m/s'};
zEst.OutputName={'yaw_rate','ay','vy'}; zEst.OutputUnit={'rad/s','m/s^2','m/s'};
zEst.ExperimentName=nameEst;
zVal.InputName=zEst.InputName; zVal.InputUnit=zEst.InputUnit;
zVal.OutputName=zEst.OutputName; zVal.OutputUnit=zEst.OutputUnit;
zVal.ExperimentName=nameVal;
split = table({files.name}',isEst,'VariableNames',{'file','estimation'});
writetable(split,fullfile(outputDir,'bag_split.csv'));
save(fullfile(outputDir,'data.mat'),'zEst','zVal','split','Ts','minSamples');
fprintf('DATA: %d estimation experiments; %d validation experiments. output=%s\n', ...
    numel(nameEst),numel(nameVal),outputDir);
options = nlgreyestOptions('Display','on','SearchMethod','lsqnonlin');
options.SearchOptions.MaxIterations = maxIterations;
fprintf('BASELINE estimation: max iterations %d\n',maxIterations);
baseline = nlgreyest(zEst,create_lateral_id_model(),options);
save(fullfile(outputDir,'baseline.mat'),'baseline','options');
starts = [0 0];
models = cell(1,1); errors = cell(1,1);
for k=1:size(starts,1)
    fprintf('FREE E start %d: Ef=%g Er=%g\n',k,starts(k,1),starts(k,2));
    try
        models{k} = nlgreyest(zEst,create_lateral_id_model_e(baseline,starts(k,:)),options);
        candidate = models{k};
        save(fullfile(outputDir,sprintf('candidate_%d.mat',k)),'candidate','options');
    catch exception
        errors{k} = getReport(exception,'extended','hyperlinks','off');
        fprintf(2,'CANDIDATE %d FAILED: %s\n',k,errors{k});
    end
end
save(fullfile(outputDir,'models.mat'),'baseline','models','starts','errors','options','-v7.3');
modelNames = {'baseline_E0','E_start_0'};
allModels = [{baseline}; models];
experiment = {}; modelName = {}; fitR = []; fitAy = []; fitVy = [];
rmseR = []; rmseAy = []; rmseVy = []; nSamples = [];
for k = 1:numel(allModels)
    if isempty(allModels{k}), continue; end
    for j = 1:numel(nameVal)
        data = getexp(zVal,j);
        [response,fit] = compare(data,allModels{k});
        predicted = response.OutputData;
        if iscell(predicted), predicted = predicted{1}; end
        if iscell(fit), fit = fit{1}; end
        rmse = sqrt(mean((data.OutputData-predicted).^2,1));
        experiment{end+1,1}=nameVal{j}; modelName{end+1,1}=modelNames{k};
        fitR(end+1,1)=fit(1); fitAy(end+1,1)=fit(2); fitVy(end+1,1)=fit(3);
        rmseR(end+1,1)=rmse(1); rmseAy(end+1,1)=rmse(2); rmseVy(end+1,1)=rmse(3);
        nSamples(end+1,1)=size(data.OutputData,1);
    end
end
scores = table(modelName,experiment,nSamples,fitR,fitAy,fitVy,rmseR,rmseAy,rmseVy);
writetable(scores,fullfile(outputDir,'validation_scores.csv'));
parameterModel = {}; parameterName = {}; parameterValue = {}; parameterFixed = [];
for k=1:numel(allModels)
    if isempty(allModels{k}), continue; end
    p = allModels{k}.Parameters;
    for j=1:numel(p)
        parameterModel{end+1,1}=modelNames{k}; parameterName{end+1,1}=p(j).Name;
        parameterValue(end+1,1)=p(j).Value; parameterFixed(end+1,1)=p(j).Fixed;
    end
end
parameters = table(parameterModel,parameterName,parameterValue,parameterFixed);
writetable(parameters,fullfile(outputDir,'parameters.csv'));
fprintf('STUDY_COMPLETE: %s\n',outputDir);
end
