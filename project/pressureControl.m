% pressure shim
% gives pressure change based on limits
function dP = pressureControl(a, p_des, p_curr)
    % conversion
    bar = @(psi) 0.0689476 * psi;
    % pressure calc
    p_gain = 10000;
    p_des = max(min(p_des, bar(a.p_max)), bar(a.p_min)); %clamp desired pressure
    dP = p_gain*(p_des - p_curr);
    dP = sign(dP) * min(abs(dP), bar(a.dp_max));
end