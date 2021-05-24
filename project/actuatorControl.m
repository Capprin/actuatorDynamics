% defines pressure control signal dependent on state
function P = actuatorControl(t,x,a)
    bar = @(psi) 0.0689476 * psi;
    Kp = 30;
    Kd = 5;
    
    P_des = bar(100) * abs(sin(t*pi/2.5)); %periodic pressure; ea. 2.5sec
    dP_des = bar(100) * (pi/2.5) * abs(-cos(t*pi/2.5));
    
    P = Kp*(P_des - x(1)) + Kd*(dP_des - x(2));
end