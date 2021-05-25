% defines pressure control signal dependent on state
function P = actuatorControl(t,x,a)
    
    Kp = 300;
    Kd = 50;
    
    bar = @(psi) 0.0689476 * psi;
    

    x_des = a.x_des(t);
    
    dx_des = a.dx_des(t);

%     x_des = (l.f(t,x)/K)-x(1);
%     
%     dx_des = 0;
    
    P = bar(Kp.*(-x_des + x(1)) + Kd.*(-dx_des + x(2)));
    
    if P<0
        P = 0;
    elseif P > bar(60)
        P = bar(60);
    end

end