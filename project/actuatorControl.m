% defines pressure control signal dependent on state
function P = actuatorControl(t,x,a)
    pascal = @(psi) 6894.76 * psi;
    if t > 2.5
        P = pascal(2);
    else
        P = 0;
    end
end