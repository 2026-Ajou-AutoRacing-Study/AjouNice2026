function [dx, y] = dynamic_bicycle_linear_tire( ...
    ~, x, u, m, Iz, lf, lr, CalphaF, CalphaR, varargin)
%DYNAMIC_BICYCLE_LINEAR_TIRE Two-DOF bicycle with linear axle tire forces.
% x = [vy; r], u = [delta; vx], y = [r; ay; vy]. Angles are in radians.
% CalphaF and CalphaR are total front/rear AXLE cornering stiffness [N/rad].
% Only the tire force is linear: atan2 and cos remain nonlinear. Measured
% vx is an input, not a longitudinal dynamic state. Exclude low-speed data.

vy = x(1); r = x(2); delta = u(1); vx = u(2);
vxMinimum = 1.0; % numerical guard only; not a low-speed vehicle model
if abs(vx) < vxMinimum
    vx = sign(vx + (vx == 0.0)) * vxMinimum;
end

alphaF = delta - atan2(vy + lf*r, vx);
alphaR = -atan2(vy - lr*r, vx);
FyF = CalphaF * alphaF;
FyR = CalphaR * alphaR;

ay = (FyF*cos(delta) + FyR)/m;
dx = [ay - vx*r; (lf*FyF*cos(delta) - lr*FyR)/Iz];
y = [r; ay; vy];
end
