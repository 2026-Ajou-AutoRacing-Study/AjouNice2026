function scores = summarize_pacejka_e_comparison(outputDir)
%SUMMARIZE_PACEJKA_E_COMPARISON Score saved baseline and single E candidate.
% Validation data is for model selection; reserve separate final-test bags.
load(fullfile(outputDir,'data.mat'),'zVal');
load(fullfile(outputDir,'baseline.mat'),'baseline');
load(fullfile(outputDir,'candidate_1.mat'),'candidate');
models = {baseline,candidate};
names = {'baseline_E0','candidate_E'};
modelName={}; experiment={}; nSamples=[]; fitR=[]; fitAy=[]; fitVy=[];
rmseR=[]; rmseAy=[]; rmseVy=[];
for k=1:numel(models)
    for j=1:numel(zVal.ExperimentName)
        data=getexp(zVal,j);
        [response,fit]=compare(data,models{k});
        predicted=response.OutputData;
        if iscell(predicted), predicted=predicted{1}; end
        if iscell(fit), fit=fit{1}; end
        err=sqrt(mean((data.OutputData-predicted).^2,1));
        modelName{end+1,1}=names{k}; experiment{end+1,1}=zVal.ExperimentName{j};
        nSamples(end+1,1)=size(data.OutputData,1);
        fitR(end+1,1)=fit(1); fitAy(end+1,1)=fit(2); fitVy(end+1,1)=fit(3);
        rmseR(end+1,1)=err(1); rmseAy(end+1,1)=err(2); rmseVy(end+1,1)=err(3);
    end
end
scores=table(modelName,experiment,nSamples,fitR,fitAy,fitVy,rmseR,rmseAy,rmseVy);
writetable(scores,fullfile(outputDir,'validation_scores.csv'));
modelName={}; parameterName={}; parameterValue=[]; parameterFixed=[];
for k=1:numel(models)
    p=models{k}.Parameters;
    for j=1:numel(p)
        modelName{end+1,1}=names{k}; parameterName{end+1,1}=p(j).Name;
        parameterValue(end+1,1)=p(j).Value; parameterFixed(end+1,1)=p(j).Fixed;
    end
end
writetable(table(modelName,parameterName,parameterValue,parameterFixed), ...
    fullfile(outputDir,'parameters.csv'));
fprintf('Validation score and parameter tables saved in %s\n',outputDir);
disp(scores);
end
