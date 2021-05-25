% defines pressure control signal dependent on state
function P = actuatorControl(t,x,a)
    bar = @(psi) 0.0689476 * psi;
    
    % gains
    Kp = 300;
    Kd = 50;
    
    P = bar(Kp.*(-a.x_des(t) + x(1)) + Kd.*(-a.dx_des(t) + x(2)));
    
    % control limits
    if P<0
        P = 0;
    elseif P > bar(60)
        P = bar(60);
    end
end