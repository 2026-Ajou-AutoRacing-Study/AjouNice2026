function model = create_lateral_id_model_e(e0Model, initialE)
%CREATE_LATERAL_ID_MODEL_E Add two free E curvature parameters to E=0 model.
% Bounds [-2,1] are pragmatic experiment constraints, not measured limits.
if nargin < 1 || isempty(e0Model)
    e0Model = create_lateral_id_model();
end
if nargin < 2
    initialE = [0, 0];
end
validateattributes(initialE, {'numeric'}, {'vector','numel',2,'finite','>=',-2,'<=',1});
values = [[e0Model.Parameters.Value], initialE(:)'];
model = idnlgrey('dynamic_bicycle_pacejka_e', [3 2 2], values, [0; 0], 0, ...
    'Name', 'Lateral bicycle Pacejka with E');
for k = 1:10
    model.Parameters(k) = e0Model.Parameters(k);
end
eNames = {'Ef', 'Er'};
for k = 11:12
    model.Parameters(k).Name = eNames{k-10};
    model.Parameters(k).Unit = '1';
    model.Parameters(k).Fixed = false;
    model.Parameters(k).Minimum = -2;
    model.Parameters(k).Maximum = 1;
end
model.InputName = e0Model.InputName; model.InputUnit = e0Model.InputUnit;
model.OutputName = e0Model.OutputName; model.OutputUnit = e0Model.OutputUnit;
model.InitialStates = e0Model.InitialStates;
end
