% defines pressure control signal dependent on state
function P = actuatorControl(t,x,a)
    bar = @(psi) 0.0689476 * psi;
    P = bar(35)*double(t>2.5);
end