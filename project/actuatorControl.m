% defines pressure control signal dependent on state
function P = actuatorControl(t,x,a)
    bar = @(psi) 0.0689476 * psi;
    P = bar(60) * abs(sin(t*pi/2)); %periodic pressure; ea. 2sec
end