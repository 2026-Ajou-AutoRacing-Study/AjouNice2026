function [candidate, scores] = compare_reference_fit(cfg, zEst, zVal, baseline, maxIterations)
% Refit using supplied independent experiments and compare held-out outputs.
% Note: d estimated on validation data leaks information; reserve a new test bag.
if nargin < 5, maxIterations=20; end
if strcmp(cfg.vySource,'position_derivative') && (~isfield(cfg,'positionDataPrepared') || ~cfg.positionDataPrepared)
    error('mpc_develop:MissingPositionData', ...
        'Build synchronized position-derived vy ID data first; current ID CSV does not contain position.');
end
if strcmp(cfg.vySource,'position_derivative'), cfg.x_ego=0; end
candidate=create_reference_id_model(cfg,baseline);
if isfinite(cfg.x_ego)
    experimentOutputs=zEst.OutputData;
    if ~iscell(experimentOutputs), experimentOutputs={experimentOutputs}; end
    lr=candidate.Parameters(strcmp({candidate.Parameters.Name},'L')).Value - ...
       candidate.Parameters(strcmp({candidate.Parameters.Name},'lf')).Value;
    vy0=zeros(1,numel(experimentOutputs)); r0=vy0;
    for k=1:numel(experimentOutputs)
        y=experimentOutputs{k}; r0(k)=y(1,1);
        vy0(k)=initial_vy_cg(y(1,3),r0(k),cfg.x_ego,lr);
    end
    candidate.InitialStates(1).Value=vy0;
    candidate.InitialStates(2).Value=r0;
end
options=nlgreyestOptions('Display','on','SearchMethod','lsqnonlin');
options.SearchOptions.MaxIterations=maxIterations;
candidate=nlgreyest(zEst,candidate,options);
scores=nan(numel(zVal.ExperimentName),2);
for k=1:numel(zVal.ExperimentName)
    data=getexp(zVal,k);
    [~,f0]=compare(data,baseline); [~,f1]=compare(data,candidate);
    scores(k,:)=[f0(3),f1(3)];
    fprintf('%d %s: vy fit baseline %.1f%% corrected %.1f%%\n',k,zVal.ExperimentName{k},scores(k,1),scores(k,2));
end
end
