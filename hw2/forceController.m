% does force control on a series-elastic actuator
function f = forceController(t, x, a, s, l)
    P = 70;
    D = 70;
    
    % get spring disp. required for f_des
    dx = a.f_des(t)/s.k;
    % get x setpoint
    x_des = l.x(t) - s.l0 - dx;
    
    % vel. setpoint is load velocity
    v_des = l.v(t);
    
    % use PD controller for forces
    f = P*(x_des-x(1)) + D*(v_des-x(2));
    
    % enforce max force
    if abs(f) > a.f_max
        f = sign(f)*a.f_max;
    end
end