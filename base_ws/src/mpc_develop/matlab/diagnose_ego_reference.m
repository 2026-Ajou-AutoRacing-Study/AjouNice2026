function result = diagnose_ego_reference(egoFile, imuFile)
% Diagnostic assumptions: position is rear axle; x forward, y left, r CCW.
% Heading is degrees in world frame; velocity frame and reference are unknown.
% A small sideslip makes the heading-direction heuristic ambiguous; do not
% treat its choice, regression, or delayed signals as physical proof.
Ts=0.02; cutoff=3; maxGap=0.15;
E=readtable(egoFile); I=readtable(imuFile);
requiredE={'t','pos_x','pos_y','heading','vel_x','vel_y','wheel_angle'};
requiredI={'t','wz','ay'};
assert(all(ismember(requiredE,E.Properties.VariableNames)) && all(ismember(requiredI,I.Properties.VariableNames)), 'Required raw columns missing; lateral-ID CSV lacks position.');
E=sortrows(E,'t'); I=sortrows(I,'t');
[~,ia]=unique(E.t,'stable'); E=E(ia,:); [~,ia]=unique(I.t,'stable'); I=I(ia,:);
assert(height(E)>10 && height(I)>10,'Insufficient samples');
t0=max(E.t(1),I.t(1)); tend=min(E.t(end),I.t(end));
t=(t0:Ts:tend)'; assert(numel(t)>30,'Insufficient overlapping time');
% Reject gaps before filtering: select the longest continuously sampled run.
eGap=interp1(E.t,[diff(E.t);0],t,'previous',NaN); iGap=interp1(I.t,[diff(I.t);0],t,'previous',NaN);
valid=isfinite(eGap)&isfinite(iGap)&eGap<=maxGap&iGap<=maxGap;
edges=diff([false;valid;false]); starts=find(edges==1); ends=find(edges==-1)-1;
assert(~isempty(starts),'No common continuous data'); [~,k]=max(ends-starts); t=t(starts(k):ends(k));
assert(numel(t)>30,'Continuous overlap too short');
psi=interp1(E.t,unwrap(deg2rad(E.heading)),t,'linear');
px=interp1(E.t,E.pos_x,t); py=interp1(E.t,E.pos_y,t);
vx=interp1(E.t,E.vel_x,t); vy=interp1(E.t,E.vel_y,t);
r=interp1(I.t,I.wz,t); ay=interp1(I.t,I.ay,t);
% Forward-backward first-order low-pass has zero phase away from edges.
px=zero_filter(px,Ts,cutoff); py=zero_filter(py,Ts,cutoff);
psi=zero_filter(psi,Ts,cutoff); vx=zero_filter(vx,Ts,cutoff); vy=zero_filter(vy,Ts,cutoff);
r=zero_filter(r,Ts,cutoff); ay=zero_filter(ay,Ts,cutoff);
pxdot=gradient(px,Ts); pydot=gradient(py,Ts);
c=cos(psi); s=sin(psi); posx=c.*pxdot+s.*pydot; posy=-s.*pxdot+c.*pydot;
worldx=c.*vx+s.*vy; worldy=-s.*vx+c.*vy;
valid=hypot(vx,vy)>3 & (t-t(1)>1) & (t(end)-t>1);
assert(nnz(valid)>20,'Insufficient moving samples');
% Small-angle speed/heading heuristic is biased by true vehicle sideslip.
worldAngle=abs(atan2(worldy(valid),worldx(valid))); bodyAngle=abs(atan2(vy(valid),vx(valid)));
scores=[median(worldAngle),median(bodyAngle)];
if scores(1)<=scores(2), frame='world'; vxb=worldx; vyb=worldy; else, frame='body'; vxb=vx; vyb=vy; end
dv=vyb-posy;
[d,rd]=fit1(r(valid),dv(valid)); [tauAy,ra]=fit1(ay(valid),dv(valid));
[tauC,rc]=fit1((vxb(valid).*r(valid)),dv(valid)); [signGain,rs]=fit1(-2*posy(valid),dv(valid));
err=[rd,ra,rc,rs]; labels={'offset d*r','timing tau*ay','timing tau*vx*r','sign-reversal proxy'}; [~,best]=min(err);
res=dv(valid)-d*r(valid); den=sum((dv(valid)-mean(dv(valid))).^2); if den>0, R2=1-sum(res.^2)/den; else, R2=NaN; end
shifts=-5:5; rmsShift=nan(size(shifts));
for j=1:numel(shifts)
 ps=interp1(t,psi,t+shifts(j)*Ts,'linear',NaN); % positive = later heading
 pvy=-sin(ps).*pxdot+cos(ps).*pydot;
 if strcmp(frame,'world'), vshift=-sin(ps).*vx+cos(ps).*vy; else, vshift=vy; end
 mask=valid & isfinite(pvy) & isfinite(vshift); if nnz(mask)>20, rmsShift(j)=sqrt(mean((vshift(mask)-pvy(mask)).^2)); end
end
[~,j]=min(rmsShift); shift=shifts(j);
low=valid & hypot(vx,vy)>=3 & hypot(vx,vy)<6 & abs(r)>0.05;
if any(low), lowMedian=median(vyb(low)./r(low)); else, lowMedian=NaN; end
fprintf('Frame heuristic: %s (world/body median direction %.3f/%.3f deg; not definitive)\n',frame,rad2deg(scores));
fprintf('d=%.4f m, R2=%.4f; heading shift=%+d samples (%+.3f s; positive uses later heading)\n',d,R2,shift,shift*Ts);
fprintf('Low-speed vy/r median=%.4f m, n=%d\n',lowMedian,nnz(low));
fprintf('RMS offset=%.4f, ay=%.4f, vx*r=%.4f, sign proxy=%.4f m/s; lowest: %s\n',err(1),err(2),err(3),err(4),labels{best});
fprintf('Coefficients: tau_ay=%.4f s, tau_vxr=%.4f s, sign proxy gain=%.4f\n',tauAy,tauC,signGain);
figure('Name','Ego reference diagnostic','Color','w');
subplot(2,2,1); plot(r(valid),dv(valid),'.'); hold on; xx=linspace(min(r(valid)),max(r(valid)),100); plot(xx,d*xx,'r','LineWidth',2); xlabel('r [rad/s]'); ylabel('dv [m/s]'); grid on;
subplot(2,2,2); plot(ay(valid),dv(valid),'.'); xlabel('IMU ay [m/s^2]'); ylabel('dv [m/s]'); grid on;
subplot(2,2,3); plot(t-t(1),vyb,'k',t-t(1),posy,'b',t-t(1),posy+d*r,'r'); legend('velocity','position derivative','position + d*r'); xlabel('Time [s]'); ylabel('vy [m/s]'); grid on;
subplot(2,2,4); plot(shifts,rmsShift,'o-'); xlabel('Heading shift [samples]'); ylabel('RMS [m/s]'); grid on;
delta=interp1(E.t,deg2rad(E.wheel_angle),t,'linear');
result=struct('frame',frame,'frameScoresRad',scores,'d',d,'R2',R2,'shiftSamples',shift,'lowSpeedMedian',lowMedian,'rms',err,'best',labels{best},'time',t,'posVy',posy,'posVx',posx,'velocityVx',vxb,'velocityVy',vyb,'yawRate',r,'ay',ay,'delta',delta);
end
function y=zero_filter(x,Ts,cutoff)
a=exp(-2*pi*cutoff*Ts); y=x; for k=2:numel(x), y(k)=a*y(k-1)+(1-a)*x(k); end
for k=numel(x)-1:-1:1, y(k)=a*y(k+1)+(1-a)*y(k); end
end
function [b,rmse]=fit1(x,y)
den=sum(x.^2); if den<1e-12, b=NaN; rmse=Inf; else, b=sum(x.*y)/den; rmse=sqrt(mean((y-b*x).^2)); end
end
