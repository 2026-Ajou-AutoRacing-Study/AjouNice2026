function [dx, y] = dynamic_bicycle_pacejka_e0( ...
    ~, x, u, m, Iz, lf, lr, Bf, Cf, Df, Br, Cr, Dr, varargin)
%DYNAMIC_BICYCLE_PACEJKA_E0 Continuous lateral dynamic-bicycle ID model.
%
% State x:
%   x(1) = vy : body lateral velocity [m/s]
%   x(2) = r  : yaw rate [rad/s]
%
% Input u:
%   u(1) = delta : physical front-wheel angle [rad]
%   u(2) = vx    : measured body longitudinal velocity [m/s]
%
% Output y:
%   y(1) = r  : yaw rate [rad/s]
%   y(2) = ay : body lateral acceleration [m/s^2]
%   y(3) = vy : body lateral velocity [m/s]
%
% The tire model is the simplified Pacejka form with E fixed to zero:
%   Fy = D*sin(C*atan(B*alpha)).
%
% This is an idnlgrey continuous-time model file. Remove samples below the
% chosen minimum vx before estimation; the vx guard below only prevents a
% singular numerical evaluation and does not make the low-speed model valid.
% idnlgrey may pass auxiliary solver arguments after the model parameters.

vy = x(1);
r = x(2);

delta = u(1);
vx = u(2);

% Dynamic bicycle slip-angle equations are invalid at standstill. Preserve
% the sign for a malformed negative sample while preventing division by zero.
vxMinimum = 1.0;  % [m/s]; preprocessing must exclude lower-speed samples.
if abs(vx) < vxMinimum
    vxSafe = sign(vx + (vx == 0.0)) * vxMinimum;
else
    vxSafe = vx;
end

% Positive alpha produces positive lateral force under this convention.
alphaFront = delta - atan2(vy + lf*r, vxSafe);
alphaRear = -atan2(vy - lr*r, vxSafe);

% Simplified Pacejka / Magic Formula: E_front = E_rear = 0.
FyFront = Df * sin(Cf * atan(Bf * alphaFront));
FyRear = Dr * sin(Cr * atan(Br * alphaRear));

% Body-frame lateral dynamics and yaw moment balance.
vyDot = (FyFront*cos(delta) + FyRear)/m - vx*r;
rDot = (lf*FyFront*cos(delta) - lr*FyRear)/Iz;

% ay = d(vy)/dt + vx*r. This form avoids differentiating measured vy.
ay = (FyFront*cos(delta) + FyRear)/m;

dx = [vyDot; rDot];
y = [r; ay; vy];
end
