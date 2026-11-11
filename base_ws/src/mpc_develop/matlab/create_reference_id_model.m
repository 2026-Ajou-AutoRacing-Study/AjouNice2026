function model = create_reference_id_model(cfg, original)
% Opt-in output reference correction. Default uses the original model unchanged.
if nargin < 1 || isempty(cfg), cfg=struct(); end
if nargin < 2 || isempty(original), original=create_lateral_id_model(); end
if ~isfield(cfg,'vySource'), cfg.vySource='velocity'; end
if ~isfield(cfg,'x_ego'), cfg.x_ego=NaN; end
if ~isfield(cfg,'x_imu'), cfg.x_imu=1.528; end
if ~isfield(cfg,'y_imu'), cfg.y_imu=0; end
if ~isfield(cfg,'applyImuOffset'), cfg.applyImuOffset=false; end
assert(any(strcmp(cfg.vySource,{'velocity','position_derivative'})),'Unknown vySource');
if strcmp(cfg.vySource,'position_derivative'), cfg.x_ego=0; end
assert(isscalar(cfg.x_ego) && (isnan(cfg.x_ego) || isfinite(cfg.x_ego)) && ...
    isfinite(cfg.x_imu) && isfinite(cfg.y_imu),'Invalid reference position');
if isnan(cfg.x_ego) && ~cfg.applyImuOffset, model=original; return; end
p=original.Parameters; names={p.Name};
idx=@(name) find(strcmp(names,name),1);
lf=p(idx('lf')).Value; lr=p(idx('lr')).Value;
order={'m','Iz','lf','Bf','Cf','Df','Br','Cr','Dr'};
values=cellfun(@(n) p(idx(n)).Value,order,'UniformOutput',false);
xEgo=cfg.x_ego; enabled=~isnan(xEgo); if ~enabled, xEgo=0; end
values=[values(1:3),{lf+lr},values(4:end),{xEgo,cfg.x_imu,cfg.y_imu,double(enabled),double(cfg.applyImuOffset)}];
model=idnlgrey('dynamic_bicycle_pacejka_e0_reference',[3 2 2],values,[0;0],0,'Name','Pacejka E=0 reference corrected');
outNames={'m','Iz','lf','L','Bf','Cf','Df','Br','Cr','Dr','x_ego','x_imu','y_imu','ego_enabled','imu_enabled'};
for k=1:numel(outNames)
 model.Parameters(k).Name=outNames{k}; model.Parameters(k).Fixed=true;
 if any(strcmp(order,outNames{k}))
   src=p(idx(outNames{k})); model.Parameters(k).Fixed=src.Fixed; model.Parameters(k).Minimum=src.Minimum; model.Parameters(k).Maximum=src.Maximum;
 end
end
lfIndex=find(strcmp(outNames,'lf'));
model.Parameters(lfIndex).Maximum=min(model.Parameters(lfIndex).Maximum,lf+lr-1e-6);
model.InputName=original.InputName; model.InputUnit=original.InputUnit;
model.OutputName=original.OutputName; model.OutputUnit=original.OutputUnit;
model.InitialStates=original.InitialStates;
end
