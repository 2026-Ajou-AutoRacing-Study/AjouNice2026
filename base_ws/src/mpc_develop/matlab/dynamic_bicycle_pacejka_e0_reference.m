function [dx,y] = dynamic_bicycle_pacejka_e0_reference(t,x,u,m,Iz,lf,L,Bf,Cf,Df,Br,Cr,Dr,xEgo,xImu,yImu,egoEnabled,imuEnabled,varargin)
% x positions relative to assumed rear axle; x forward, y left, r CCW.
% L fixed separately so lr tracks changes in lf during estimation.
lr = L-lf;
[dx,y] = dynamic_bicycle_pacejka_e0(t,x,u,m,Iz,lf,lr,Bf,Cf,Df,Br,Cr,Dr);
if egoEnabled, y(3) = x(1) + (xEgo-lr)*x(2); end
if imuEnabled, y(2) = y(2) + (xImu-lr)*dx(2) - yImu*x(2)^2; end
end
