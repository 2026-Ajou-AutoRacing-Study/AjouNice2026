%% Linear-tire bicycle model: same estimation/validation split as the E study
% Run as sections after setting resultDir. This script deliberately leaves the
% previous workspace intact; it does not overwrite the saved Pacejka models.
modelFolder = fileparts(mfilename('fullpath'));
addpath(modelFolder);
resultDir = '/home/kdy/AjouNice2026/planner_logs/analysis/mpc_develop/pacejka_e_comparison_20261108_trial';
loadedData = load(fullfile(resultDir,'data.mat'),'zEst','zVal');
loadedBaseline = load(fullfile(resultDir,'baseline.mat'),'baseline');
zEstLinear = loadedData.zEst;
zValLinear = loadedData.zVal;
pacejkaE0 = loadedBaseline.baseline;

%% Estimate per-axle cornering stiffness using the existing training bags
initialLinearModel = create_linear_tire_id_model();
linearOptions = nlgreyestOptions('Display','on','SearchMethod','lsqnonlin');
linearOptions.SearchOptions.MaxIterations = 20;
linearModel = nlgreyest(zEstLinear,initialLinearModel,linearOptions);
present(linearModel)

%% Compare both models on the same held-out experiment
% Experiment 5 is full_source_mix_037_seg4 in the saved split; inspect
% zValLinear.ExperimentName before selecting another experiment.
validationExperiment = 5;
validationDataLinear = getexp(zValLinear,validationExperiment);
figure('Name','Pacejka E=0 versus linear tire');
compare(validationDataLinear,pacejkaE0,linearModel);
