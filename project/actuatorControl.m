% defines pressure control signal dependent on state
function P = actuatorControl(t,x,a)
    
    Kp = 30;
    Kd = 5;
    
    bar = @(psi) 0.0689476 * psi;
    
%     P_des = bar(100) * (1+sin(t*pi/2.5)); %periodic pressure; ea. 2sec
    
%     dP_des = bar(100) * (pi/2.5) * (cos(t*pi/2.5));
    
    x_des = a.x_des(t);
    
    dx_des = a.dx_des(t);
    
    P = 100.*Kp.*(-x_des + x(1)) + 100.*Kd.*(-dx_des + x(2));
    
    if P<0
        P = 0;
    end

%     P = bar(-900);
    
%     if t<3
%         P = bar(5000);
%     else
%         P = bar(0);
%     end
end