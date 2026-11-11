function vyCG = initial_vy_cg(vyMeasured, yawRate, xEgo, lr)
% Convert a point's velocity to the CG; NaN retains the previous assumption.
if isnan(xEgo), vyCG=vyMeasured; else, vyCG=vyMeasured-(xEgo-lr).*yawRate; end
end
