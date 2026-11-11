function data=position_reference_iddata(diagnostic, name, Ts)
% Convert one diagnostic output into independent continuous ID experiments.
% Assumes input diagnostic position is rear axle; velocity vx convention is
% unchanged. Discard filter edges and avoid connecting low-speed gaps.
if nargin<3, Ts=0.05; end
t=(diagnostic.time(1):Ts:diagnostic.time(end))';
u=[interp1(diagnostic.time,diagnostic.delta,t),interp1(diagnostic.time,diagnostic.velocityVx,t)];
y=[interp1(diagnostic.time,diagnostic.yawRate,t),interp1(diagnostic.time,diagnostic.ay,t),interp1(diagnostic.time,diagnostic.posVy,t)];
keep=hypot(u(:,2),y(:,3))>=3 & t>t(1)+1 & t<t(end)-1 & all(isfinite([u y]),2);
edges=diff([false;keep;false]); starts=find(edges==1); ends=find(edges==-1)-1;
inputs={}; outputs={}; names={};
for k=1:numel(starts)
    if ends(k)-starts(k)+1<40, continue; end
    inputs{end+1}=u(starts(k):ends(k),:); %#ok<AGROW>
    outputs{end+1}=y(starts(k):ends(k),:); %#ok<AGROW>
    names{end+1}=sprintf('%s_seg%d',name,k-1); %#ok<AGROW>
end
assert(~isempty(inputs),'No usable moving segments');
data=iddata(outputs,inputs,Ts);
data.InputName={'delta','vx'}; data.InputUnit={'rad','m/s'};
data.OutputName={'yaw_rate','ay','vy'}; data.OutputUnit={'rad/s','m/s^2','m/s'};
data.ExperimentName=names;
end
