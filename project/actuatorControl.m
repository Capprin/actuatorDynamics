% defines pressure control signal dependent on state
function P = actuatorControl(t,x,a)
    bar = @(psi) 0.0689476 * psi;

    % do derivative control for force
    k = 1; % don't know how this works, don't change it
    % TODO: shift desired force somehow?
    dx_des = -a.f_des(t) / k;
    
    % gain and position control
    baseline = 20; % starting pressure
    Kd = 10; % tunes response amplitude
    P = bar(baseline + Kd.*(dx_des - x(2)));
end