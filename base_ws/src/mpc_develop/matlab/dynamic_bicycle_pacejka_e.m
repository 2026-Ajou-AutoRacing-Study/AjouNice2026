function [dx, y] = dynamic_bicycle_pacejka_e(~, x, u, m, Iz, lf, lr, Bf, Cf, Df, Br, Cr, Dr, Ef, Er, varargin)
%DYNAMIC_BICYCLE_PACEJKA_E Lateral bicycle with independent curvature factors.
vy = x(1); r = x(2); delta = u(1); vx = u(2);
if abs(vx) < 1.0
    vx = sign(vx + (vx == 0)) * 1.0;
end
alphaF = delta - atan2(vy + lf*r, vx);
alphaR = -atan2(vy - lr*r, vx);
zf = Bf * alphaF;
zr = Br * alphaR;
FyF = Df * sin(Cf * atan(zf - Ef*(zf - atan(zf))));
FyR = Dr * sin(Cr * atan(zr - Er*(zr - atan(zr))));
ay = (FyF*cos(delta) + FyR)/m;
dx = [ay - vx*r; (lf*FyF*cos(delta) - lr*FyR)/Iz];
y = [r; ay; vy];
end
